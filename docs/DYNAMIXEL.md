# Dynamixel notes

Working notes on the parts of the ROBOTIS Protocol 2.0 servo family this
framework implements, and on the details that cost time to get right. The
authority is the ROBOTIS e-Manual control tables; this file records what the
implementation depends on and why.

## A profile is a control table plus its unit scales

`Profile` in `profile.rs` carries both halves of what a model needs: the
register addresses, and what the counts in those registers mean. The addresses
are largely shared across Protocol 2.0 — the scales, the supply range and the
speed are not, and neither is whether a register exists at all.

Every number below is from the ROBOTIS e-Manual page for that model.

| profile | models | current | supply (rated) | no-load speed |
|---|---|---|---|---|
| `x` | XM430, XH430, XM540 | 2.69 mA/count | 10 – 14.8 V (12) | family bound, 4 rev/s |
| `xl330` | XL330-M077, XL330-M288 | 1 mA/count | 3.7 – 6.0 V (5.0) | 103 rev/min |
| `mx64` | MX-64 (2.0), model 311 | 3.36 mA/count | 10 – 14.8 V (12) | 63 rev/min |
| `mx106` | MX-106 (2.0), model 321 | 3.36 mA/count | 10 – 14.8 V (12) | 45 rev/min |
| `mx28` | MX-28 (2.0), model 30 | **none** | 10 – 14.8 V (12) | 55 rev/min |
| `xl430` | XL430-W250, 2XL430-W250 | **none** | 6.5 – 12.0 V (11.1) | 57 rev/min |
| `xc430` | XC430-W150, XC430-W240 | **none** | 6.5 – 14.8 V (12) | 70 rev/min |

Driving one through the other's profile is not a visible failure. The servo
still moves, and it holds position, and nothing in the logs looks wrong. What
changes is that every current reading is out by some factor, which makes a
current trip meaningless, and the supply range belongs to a different class of
motor.

## Half the family has no current sensor at all

This is the trap the table above exists for, and it is worse than a wrong
scale. On an MX-28, an XL430, a 2XL430 or an XC430 the register at **address
126 is Present Load** — a signed value in units of 0.1 % of PWM duty, range
−1000 to +1000 — and not Present Current. There is no Goal Current at 102
either, and `Operating Mode(11)` offers only velocity (1), position (3),
extended position (4) and PWM (16): no current control, no current-based
position control.

Read through a current-sensing profile, a servo pushing at 25 % duty returns
250 counts and reports itself as drawing 0.67 A. The number is plausible, it
tracks load, and it is not a current. So `has_current_sense` on the profile
gates three things:

- `q_current` and `torque` are reported as `None` rather than as a scaled load
  percentage. Position, velocity, voltage and temperature still decode; the
  claim is about one register, not the servo.
- `Mode::Torque` is refused at `operating_mode_for`, naming the profile, rather
  than writing mode 0 to a motor that does not have it.
- The torque command path refuses before writing to a Goal Current register
  that is not there.

The e-Manual is the authority. When adding a model, check address 126 on its
page before assuming it matches its neighbours — XL430 and XM430 differ here
despite sharing almost the whole control table.

## The MX family is Protocol 2.0 only

`known_model` lists MX-28 (2.0) as 30, MX-64 (2.0) as 311 and MX-106 (2.0) as
321. The Protocol 1.0 models report different numbers and are deliberately
absent: this build speaks only the packet format in `protocol.rs`, so a
Protocol 1.0 servo will not answer at all, and listing it here would promise
otherwise. An MX that pings as unknown is usually one that needs its firmware
switched to 2.0 in Dynamixel Wizard, not a wiring fault.

Torque is still reported as unmeasured on the MX profiles even though they
measure current: `torque_constant` is `None`, because the e-Manual gives stall
torque and stall current rather than a motor constant, and a ratio derived from
those two is not the constant. Set it explicitly to get torque in newton-metres.

## A chain is cycled as a whole, not a servo at a time

One port, one wire, and one link per joint that each wanted a round trip per
control period. That works to about eight joints and then stops working, and it
does not fail in a way that points at itself. Measured on this bench, a
sixteen-servo chain at 1 Mbaud:

| | before | after |
|---|---|---|
| joints that energised, 15 asked for | 0 | 15 |
| joints that energised, 16 asked for | 0 | 16 |
| per-joint rate, 8 joints at 10 Hz | 0–5 Hz | 10 Hz |
| aggregate transactions a second | ~22 | one write + one read a period |

The failure looked like nothing at all: `bringup` reported that the joints had
not energised and every one of them gave an empty reason, because preflight had
nothing to complain about — there had simply not been enough cycles for a joint
to arm. Raising the arming budget twelvefold did not help, which is what ruled
out a race and pointed at throughput.

`chain.rs` owns the period now. Every goal position goes out in one sync write,
every present-value block comes back in one sync read, and whichever link finds
the readings stale performs that transaction for the whole bus while the others
wait on the mutex and then find the answer already taken. Arming, mode changes,
the bus watchdog and identification stay per-servo: they are rare, and they are
not what the period is made of.

Two details cost a bench session each:

- **A sync read's replies arrive together.** The servos answer back to back, so
  a single `read` very often returns two or three status packets concatenated.
  Reading one packet into a buffer that starts empty each time returns the first
  and discards the rest, and the chain then reports that only its first servo
  answered — which reads as fifteen servos having gone quiet. The reply buffer
  now carries across the whole transaction and each packet is drained from it.
- **The port is held across the request and every reply.** Releasing it between
  them lets another thread's request land in the middle of the reply stream,
  after which both transactions read each other's answers and both time out.

The load line in `bringup -check` follows the same rule: a CAN bus is charged a
transaction per joint per cycle, a Dynamixel chain two a period however many
servos are on it. The same sixteen joints that used to be reported as 160
transactions a second are 20.

Two things still bound this. The bring-up is reliable to 16 joints at 10 Hz and
starts losing the arming race at 25; and the per-servo writes that arming needs
— torque enable, the profile registers, the watchdog — still go one at a time,
so it is the burst at connect rather than the steady state that sets the
ceiling.

## What the chain's period costs, and the three things taken off it

The chain made the steady state two transactions a period. But three costs
still scaled with the servo count, and each undid part of that on the wire.
계산값(1 Mbaud, 16 servos); 실물 16축 체인은 벤치에 없어 단일 XL330 실측 +
회선 계산으로 외삽한 값이다.

- **The hardware-error read was a round trip per servo.** `read_telemetry`
  read register 70 one servo at a time on every non-fast query, which turned
  the chain's O(1) read back into O(N) — sixteen extra round trips, about
  17.6 ms, on every full-query period. Faults latch in the servo, so they do
  not need reading every cycle: off a chain the read now rides only the rare
  verify cadence (about once a second), and on a chain it travels as one more
  broadcast — a single one-byte sync read of register 70 for the whole bus,
  cached between fault periods. Sixteen round trips became one.

- **Return Delay Time was never set.** Register 9 ships from the factory at
  250 — half a millisecond the servo waits before every status packet — and
  the code never touched it. On a sync read those pauses fall in series, one
  per servo, so sixteen servos spend about 8 ms a period waiting for silence.
  `open` now normalises it: read first, and only if it is wrong disable torque,
  write the target, and wait out the EEPROM settle — so a servo already
  normalised costs one read and nothing else, and only its first-ever bring-up
  is slow. 미검증: the target is 0 (answer immediately); it must be confirmed
  on the bench that this adapter family does not drop the first reply bytes at
  0 before the constant is trusted — raise to 2–4 counts if it does.
  (2026-09-04 확정: 0 은 3 Mbaud 15축에서 응답 헤더를 깨뜨린다. 지금 값은 10(20 µs).
  아래 "15축 sync read 꼬리 유실" 섹션.)

- **The wide block rode every period.** A chain read all 21 bytes — through
  voltage and temperature — every period, eleven more per servo than the fast
  ten-byte block, about 1.8 ms a period on sixteen servos. Voltage and
  temperature move on thermal timescales, so the chain now rotates: the fast
  block most periods, the wide block every tenth — the same bargain the
  session's own full-query cadence makes for a servo off a chain.

Together these take a sixteen-servo period from about 14.5 ms (wide block plus
factory return delays) toward about 4.6 ms — roughly 69 Hz to something over
200 Hz, on the wire arithmetic. Confirm on the bench before quoting it.

## One Driver per bus, and why individual writes had to leave the bus

A sixteen-servo UBP arm at 3 Mbaud would not arm above about ten joints: most
servos e-stopped on `Data range error (0x04)` writing profile velocity (112),
the rest on `comm_loss`. One servo alone was flawless — the tell that this was
contention, not a bad value.

The cause was two Drivers per servo over one port. Each `DynamixelLink` built
its own `Driver`; the chain built another; all wrapped clones of the same
`SharedPort`. The port mutex serialised transactions correctly, but
`Driver::needs_flush` is per-instance, so a failure on one Driver never flushed
the next Driver's read — and only telemetry and the goal position went through
the chain. Every mode, limit, torque and non-position goal write was an
*individual* answered transaction on the link's own Driver, sixteen threads
contending, their status replies read against the wrong request.

Two moves fixed it. First, a link on a chain adopts the chain's Driver
(`Chain::driver`), so the whole bus shares one `needs_flush`. Second — and this
is what actually cleared the 0x04 — the hot-path RAM writes were queued into
`Chain::cycle` the way the goal position already was: torque, profile
velocity/accel, bus watchdog and the velocity/current goals each become one
broadcast `sync_write` across every servo, emitted in a fixed phase order
(torque-off, limits ascending, torque-on, goals) that preserves the arm
sequence. A broadcast is unanswered, so there is no status reply left to be
misread. EEPROM writes — operating mode, return delay — stay discrete off the
cycle: their settle is a sleep between two locked transactions, not inside one,
so they never freeze the bus for the other joints, and folding them into the
sync cycle is the one thing that would.

## A long sync read loses its tail

With the writes fixed, the last few joints still dropped out — `comm_loss` on
the servos past the twelfth, on both the fast and the wide read. A single sync
read of sixteen servos reliably returned the first twelve replies and timed out
on the thirteenth; a shorter request answered in full. 확인함 (UBP bench,
3 Mbaud): asking in chunks of eight (`MAX_SYNC_READ`) — two sync reads instead
of one — every servo answered, every period. The tail loss on the longer
stream is a physical-layer limit of the daisy chain, not the protocol; keeping
each reply stream short works around it while staying O(1) in transactions per
period for a chain this size.

With all three in place a fifteen-joint UBP arm (the sixteenth, an XL430 wrist,
sits above its rated 12 V supply and e-stops on over-voltage — an electrical
issue, not this one) armed and held at 100 Hz with about 95 Hz achieved, and
swept to roughly 200 Hz before the wire became the ceiling — against a bus that
would not previously arm the machine at ten.

## The servo says which one it is, so the profile is never guessed

A Dynamixel answers every ping with its model number. `known_model` maps that
number to the profile that describes it, and `resolve_profile` settles the
question at open time:

- **Nothing declared, model recognised** — the model's profile is used. A
  config file that forgets `"profile"` is still driven correctly.
- **Declared, and the model says otherwise** — refused, naming both. One of the
  two is wrong, and connecting anyway means reporting numbers scaled for a
  motor that is not on the bus.
- **Model not in the table** — the configured profile stands, but `Identify`
  reports the board as unrecognised unless a profile was named. The choice is
  then acknowledged rather than assumed, which is the same bargain the moteus
  driver strikes with an unfamiliar board revision.

Before this, the model number was read and printed and then ignored: an XL330
fell back to the X-series table and the board still reported itself as
`supported`.

## The rated supply has to be published, not just known

The host safety layer opens with a bus voltage band of 10–42 V, which is a
sensible band for a 12 V or 24 V bus and an impossible one for a 5 V servo.
`safety.FitVoltageTo` exists to widen a band that excludes the connected
motor's rated supply — a band that excludes it catches no fault, it just trips
the instant the machine is powered.

That check keys on `Envelope.voltage_rated`, and it does nothing when the field
is zero. The Dynamixel profiles carried a min and a max but no rated value, so
the check never fired and an XL330 reporting a perfectly healthy 4.8 V could
not be armed at all:

```
Cannot arm:
  ✗ Bus voltage 4.8 V is below the configured minimum 11.0 V.
```

Every profile now declares `voltage_rated`, and a test fails if one does not.

## The serial read timeout was the loop rate

A serial port opens with a plain total read timeout — 10 ms — and a plain total
timeout means a read blocks until it has filled the *whole* buffer it asked for
or the timeout expires. `readPacketWithTimeout` asks for a buffer far larger
than any status packet, so it never fills, and every read cost the full 10 ms
however quickly the servo answered.

Measured on an XL330 at 1 Mbaud:

| | per cycle | rate |
|---|---|---|
| as the port opens | 15.01 ms | 66.6 Hz |
| with a 1 ms read timeout | 1.12 ms | 889.8 Hz |

`Link.Open` now asks the port for a 1 ms read timeout through the optional
`serial.ReadTimeouter` interface, which uses the Windows combination that
returns as soon as any byte arrives. A 100 Hz control loop went from a measured
70.0 Hz with 5.92 ms of jitter to 99.9 Hz with 0.44 ms.

This is the serial half of the same mistake the CAN side had; see the matching
section in [MOTEUS.md](MOTEUS.md).

## The bus watchdog latches, and a killed session leaves it latched

The bus watchdog is the servo's own safety net: set register 98 to a timeout,
stop sending, and the motor stops by itself.  The framework sets it every
cycle, which is right.

What is easy to miss is what happens after it fires. The register latches at
0xFF, and while it is latched the servo refuses **every goal register** —
position, velocity, current, PWM — with an access error, code 0x07.  Profile
velocity, profile acceleration and the LED still accept writes, which is what
makes the state so confusing to read from the outside: the servo answers,
reports no hardware error, identifies itself, arms, and then rejects the first
command.

A session that ends without releasing the servo leaves exactly that state: a
killed process, a crash, a pulled cable.  Before `Open` cleared it, the next
bring-up would come up clean and then e-stop with

```
error j1  E-STOP · Transport error
          dxl error code: 07
```

and stay that way until somebody wrote a zero to a register they had never
heard of.  Taking control of a servo means clearing what the last session left,
the same way the moteus driver forgets its sticky registers on connect.

Status packet error codes are decoded rather than printed as a number, because
`07` names nothing anyone can act on and the cause is nowhere near the symptom.

## Telemetry is split across two query profiles

Position, velocity and current are one 10-byte read from 126. Voltage,
temperature and the hardware error register cost extra bytes and an extra
transaction, so they ride only on the wider profiles — a fast cycle leaves them
unreported rather than stale, and `Telemetry.Merge` keeps the last known value.

## Torque is not reported unless the constant is known

`Profile.TorqueConstant` is zero for the families here, because it varies per
model within a family. A zero means torque is reported as unmeasured rather
than as a number derived from a guess. Set `torque_constant` in the config to
get real torque readings.

## Verified

Against an XL330-M288-T (model 1200, firmware 53) on COM6 at 1 Mbaud:
open and identify, profile resolution in all three cases above, position
control, velocity control, the fast/full telemetry split, 885 cycles/s with no
failures, and `bringup` from a robot description with the host envelope
clamping a 99 rad request to the joint's 5.5 rad limit.

## 해결: "sync write goal 이 서보에 반영되지 않는다" 는 latch 된 bus watchdog 이었다 (2026-09-04)

2026-09-03 에 미해결로 남긴 증상 — 15축 UBP 를 base 포즈로 이동시키면 chain 의 sync
write(0x83) 로 보낸 goal 을 서보가 무시하고 Goal Position(116) 이 현재 위치에 머문다,
개별 write(0x03) 로 바꾸면 0x04 data-range-error 가 난다 — 는 패킷 문제가 아니라
**bus watchdog(98) 이 latch 된 서보의 정상 동작**이다. 벤치(COM7 3 Mbaud, MX-64 id 1)
에서 `examples/dxl_diag.rs` 로 층별 실측:

- 토크 on 상태에서 goal = present+100 을 개별 write, 1축 sync write, 16축 sync write
  로 각각 보내면 **셋 다** 116 에 반영되고 서보가 움직인다. sync write 패킷은 정상.
- watchdog 을 20(400 ms) 으로 켠 뒤 100 ms 마다 sync read / sync write / 개별 read
  만 보내면 어느 쪽이든 watchdog 이 리셋되어 20 에 머문다. **아무것도 보내지 않으면**
  2 s 뒤 98 = 255(latch) 가 되고, 그 상태에서 sync write goal(present+50) 은 116 에
  반영되지 않고 present 값이 남으며, 같은 값을 개별 write 하면 `Data range error
  (0x04)` 로 거부된다. XL330 은 같은 상태를 0x07 로 답했다(위 섹션) — 코드는 모델마다
  다르고 의미는 같다.
- 토크 off 인 서보는 116 이 present 를 따라가므로 goal 반영 여부를 판정할 수 없다.
  진단은 토크 on 에서 해야 한다.

즉 지난 세션의 "큰 변화만 안 된다" 는 latch 된 뒤 present 가 조금씩 흐르는 것을 goal
반영으로 오독한 것이고, latch 는 디버깅 중 어느 시점(개별 write 실험, 프로세스 정지)에
생긴 뒤 세션이 이어지는 동안 아무도 알아채지 못한 것이다. sync write 는 응답이 없어
거부를 들을 수 없고, 코드에 98 을 읽는 곳이 `open` 의 clear 외에 없었다.

고친 것:

- **chain 이 fault 주기(1 s)마다 98 을 sync read** 한다(`Chain::set_watchdog_addr`,
  `Chain::watchdog`). 링크는 `read_telemetry` 앞에서 0xFF 를 보면 98 에 0 을 쓰고
  `last_watchdog` 을 지운 뒤 `Error::Refused` 로 원인을 적어 돌려준다 — 세션은
  `link_error` e-stop 으로 "the bus watchdog fired and latched" 를 표시하고, 재arm 시
  timeout 을 다시 쓴다. 조용히 안 움직이는 서보가 원인이 적힌 정지가 된다.
- **`close` 가 토크 off 를 driver 로 직접 쓴다.** 전에는 `chain.leave` 뒤에
  `set_torque(false)` 를 불러 chain 큐에(이미 빠진 멤버라 no-op) 넣었고, 루프가 이미
  멈춘 뒤라 어차피 방출될 사이클도 없었다. 실측: bringup 종료 후 16축 전부 torque=1.
  수정 후 0.
- `DRLABS_DXL_TRACE=1` 로 chain 이 주기 2배 이상의 공백과 latch 된 id 를 stderr 에
  적는다.

확인(2026-09-04, 15축 `ubp.json` 에서 id 11 제외, 10 Hz): panel `POST /api/move
{"pose":"base","velocity":0.5,"accel":1.0}` → 14 관절 progress 1, 목표 ±0.03 rad 이내
도달. r_sh_p(id 2) 만 이동 중 버스 전압 9.9 V < MX-64 프로필 하한 10.0 V 로 81% 에서
e-stop — 공급이 부하에서 처진다. 전원/배선 또는 관절별 `bus_voltage_min` 의 문제이지
드라이버가 아니다. 같은 실행에서 watchdog latch 도, 주기 공백도 추적에 찍히지 않았다.

주의: `bringup -sweep f` 의 중심은 현재 위치가 아니라 **각 관절 범위의 중앙**이다.
`-sweep 0.05` 도 먼저 범위 중앙까지 이동한 뒤 ±2.5 % 를 왕복하므로, 작은 값이라고
작게 움직이지 않는다.

## 15축 sync read 꼬리 유실 — 1바이트 broadcast 와 Return Delay 0 (2026-09-04)

같은 날, base 포즈를 유지하던 중 id 6·7·8(r_sh_y, l_elb_p, r_elb_p) 이 "5 사이클 연속
무응답" 으로 e-stop 해 힘이 빠졌다(sweep 중에도 7·8 이 같은 식으로). 청크 8 의 fast/wide
읽기는 단독 400 사이클 무손실이라 그 자체는 아니었다. `examples/dxl_diag.rs`
`syncstress` / `raw` 로 실측(COM7 3 Mbaud, 15축, Return Delay 0):

| sync read | 결과 |
|---|---|
| 8축 × 10 바이트, 토크 on 400 사이클 | 손실 0 |
| 8축 × 21 바이트(wide) | 손실 0 |
| 9축 × 21 바이트 | 손실 0 |
| 12·13축 × 10 바이트 | 12번째 응답부터 유실, 사이클의 69 % |
| 15축 × 10 바이트 | id 13~16 유실 68 % |
| **15축 × 1 바이트** (chain 의 fault 읽기가 이것) | id 3 이후 거의 전부 유실, 사이클 93 % 가 100 ms timeout |

원시 덤프를 보면 응답마다 앞에 잡음 바이트 `00` 이 붙고(회선 turnaround), RDT 0 에서는
헤더 첫 바이트가 깨진 응답(`FD FF FD 00 03 …`) 이 여섯 중 둘, 그 뒤 차례를 기다리던
서보들은 아예 응답하지 않는다. RDT 를 10(20 µs) 으로 바꾸면 헤더 손상은 사라지고 15축
1바이트 읽기가 15개 전부 돌아오는 경우가 생기지만, 여전히 절반쯤은 12번째 응답(id 13,
XL330) 부터 블록으로 빠진다 — 위치 기준이며 바이트·시간 기준이 아니다. 원인은 서보
펌웨어 쪽으로 추정만 하고 확정하지 않았다. 청크 8 이하로는 어떤 길이에서도 유실이 없다.

세션에서 e-stop 이 된 경로: chain 의 fault 읽기(reg 70)와 새로 넣은 watchdog 읽기(reg 98)
는 15축을 **한 패킷**으로 물었고, 그 응답이 잘리면 100 ms timeout 을 두 번 — 100 ms 주기
한가운데 200 ms 정지가 1 초마다. 게다가 `sync_read` 가 `for _ in ids` 로 응답 개수를
셌기 때문에, 잘린 broadcast 에 늦게 답한 다른 id 의 응답이 다음 읽기의 슬롯을 소모해 그
청크의 꼬리(6·7·8)가 "무응답" 이 됐다.

고친 것: (1) fault·watchdog 읽기도 `MAX_SYNC_READ` 로 청크 분할, (2) `sync_read` 는 요청한
id 가 다 답하거나 timeout 까지 수집하고 목록 밖 응답은 슬롯을 소모하지 않음, (3)
`RETURN_DELAY_COUNTS` 0 → 10(20 µs, EEPROM 이라 open 때 다르면만 씀). 확인: panel 로 base
포즈 유지 3 분 soak, 15관절 armed 유지, 목표 오차 ≤ 0.032 rad, e-stop 0, 주기 공백 0.

## 청크 꼬리 유실로 인한 comm_loss e-stop, 그리고 Alert 비트 (2026-09-05)

원 그리기 데모(`tools/demo_circle.py --shared`, panel 10 Hz, 15축 COM7 3 Mbaud)를 20분쯤
돌리던 중 00:12:23 에 id 5·6·7·8(l_sh_y, r_sh_y, l_elb_p, r_elb_p — 첫 sync read 청크
id 1~8 의 꼬리 4개이자 그 청크의 MX-28 전부)이 "5사이클 연속 무응답" 으로 e-stop, 토크가
끊겨 팔꿈치가 떨어졌다(-1.5 → +0.07 rad). 자세는 원 꼭대기(sh_p ≈ -0.06, 팔꿈치 -1.5 rad,
부하 최대). 직전까지 panel 폴러(4 Hz)는 GET 지연 ≤ 27 ms, 루프 10.00 Hz, 팔 오차 ≤ 0.2 rad
였고 e-stop 직후에도 그대로였다 — 호스트 쪽 지연이 아니다. 그 전 "한 번씩 멈칫" 하던
구간(데모 로그에서 팔·그리퍼·머리 오차가 같이 0.5 rad 까지 올랐다 복귀, 6~13 s)은 같은
유실이 5사이클 미만으로 끝난 것으로 추정한다(증명 안 됨).

panel 을 내리고 바로 `dxl_diag dump 5,6,7,8,1,9`: **hwerr(70) = 0, wdog(98) = 0** 전부.
Hardware Error Status 는 재부팅 전까지 남으므로, 응답이 Alert 비트 때문에 거부된 것이
아니라 실제로 **응답이 오지 않았다**(또는 서보가 재부팅됐다 — 구분 못 함). 청크 8 은
2026-09-04 벤치에서 400 사이클 무손실이었지만, 부하가 걸린 채로는 꼬리 4개가 0.5 s 이상
빠질 수 있다는 뜻이다. 원인(EMI·전원·서보 펌웨어)은 모름. 다음 단서는
`DRLABS_DXL_TRACE=1` 로 띄운 panel 의 `[dxl-chain]` 줄과 로직 애널라이저다.

그래도 코드에서 잘못돼 있던 것 하나를 이 김에 고쳤다. Protocol 2.0 상태 패킷의 error
필드는 bit 7 이 **Alert**(하드웨어 오류 레지스터가 0 이 아닌 동안 모든 응답에 세팅), bit
0~6 이 명령 오류다. `sync_read` 는 `error == 0` 인 응답만 받았으므로 입력전압·과부하 같은
하드웨어 오류가 뜬 서보는 **멀쩡히 응답해도 무응답**이 됐고, 그 상태를 알려줄 reg 70
읽기도 같은 경로라 화면에는 영원히 "No fault" 가 남았을 것이다. 지금은:

- `sync_read_alerts` 가 Alert 만 선 응답을 답으로 받고 그 id 를 따로 돌려준다. 명령 오류
  비트가 선 응답은 여전히 버린다. `exchange`(단일 트랜잭션)도 같은 규칙.
- chain 이 멤버마다 `alert` 를 기억하고, link 는 reg 70 이 아직 안 읽혔어도 "Hardware
  alert" 를 fault 로 올린다(hard). 다음 fault 케이던스에 실제 비트로 바뀐다.
- `sync_read` 의 수집 루프에 건너뛴 패킷 상한(`MAX_SKIPPED_REPLIES` × 요청 수)을 뒀다.
  늦은 응답·거부 응답은 슬롯을 채우지 않으면서 timeout 을 새로 잡았기 때문에, 그런
  패킷이 이어지면 한 번의 sync read 가 100 ms 를 한참 넘길 수 있었다.

확인: `cargo test -p drlabs-dynamixel` 67개 통과(Alert 수용·거부·상한·단일 exchange 4개
추가). 실기는 새 바이너리로 panel 재기동 후 원 3회전 e-stop 0, trace 줄 0, base 복귀 오차
0.09 rad. 이어서 무한 반복 soak(아래 참조).

데모 쪽: 사용자가 원이 너무 앞에 그려진다고 해서 `--shared-cx/--shared-cz` 기본을
0.38/0.38 → 0.28/0.28 로 당겼다. 손-어깨 거리가 같으면 팔꿈치 범위가 같으므로 앞뒤·상하를
함께 당겨야 팔꿈치 한계(-1.617) 안에 남는다(sh_p -0.49 .. +0.09, elb_p -1.50 .. -0.49).
sh_p 는 음수가 앞이다(2026-09-04 실기 확인, 이전 가정의 반대).

## 청크 꼬리 유실 방지 — 응답 예산 20 ms 와 놓친 서보 개별 재읽기 (2026-09-05)

앞 절의 유실이 "발작" 으로 보이는 경로를 코드에서 끊었다. 유실 자체(EMI·전원·펌웨어)는
아직 원인을 모르지만, 유실 한 번이 버스 정지와 comm_loss e-stop 으로 번지는 두 고리는
코드였다.

1. **한 청크에 무응답 id 가 하나라도 있으면 그 청크가 100 ms 통째로 멈췄다.** sync read
   의 응답 하나하나가 `Driver::timeout`(트랜잭션 타임아웃 100 ms) 을 기다렸기 때문이다.
   16축 두 청크에 각각 하나씩 빠지면 블록 읽기만 200 ms, fault 케이던스에 걸리면 그
   위에 400 ms — 그 사이 goal 이 안 나가고, 재개되면 서보가 밀린 목표를 프로파일 속도
   1.5 rad/s 로 한 번에 따라잡는다. 이제 sync read 의 응답 대기와 아래 복구 읽기는
   `Driver::reply_timeout`(기본 `DEFAULT_REPLY_TIMEOUT` = 20 ms) 을 쓴다. 8개 응답이
   3 Mbaud 에서 1 ms 남짓에 도착하므로 20 ms 늦은 응답은 오지 않는 응답이다. 단일
   트랜잭션(EEPROM 쓰기 등) 은 100 ms 그대로다.
2. **놓친 id 를 다시 묻지 않았다.** sync read 에서 서보는 앞 서보의 응답을 보고 차례로
   답하므로 하나가 빠지면 뒤가 전부 침묵한다(청크 8 에서 id 5~8 이 함께 빠진 이유로
   추정). 이제 `Chain::recover_silent` 가 블록 읽기 직후 답 없는 서보를 `read_quick`
   (개별 READ, reply_timeout 예산, Alert 비트 수용) 으로 한 번 더 묻고, 답하면 그
   사이클은 정상 응답으로 친다. 연속 `RECOVERY_GIVE_UP`(5) 사이클 침묵한 서보는
   broadcast 에 다시 답할 때까지 개별 읽기를 건너뛴다 — 죽은 서보 하나가 매 주기
   다른 관절의 시간을 먹지 않게.

트레이스(`DRLABS_DXL_TRACE=1`) 에 줄이 하나 늘었다. broadcast 에 침묵한 id 가 있을 때만:

```
[dxl-chain] silent on the broadcast [5, 6, 7, 8]: recovered [5, 6, 7, 8], lost [], given up [] (3.2 ms for the read phase)
```

`recovered` 가 채워지면 유실은 있었지만 관절은 사이클을 놓치지 않은 것이고, `lost` 가
같은 id 로 반복되면 그 서보(또는 그 앞의 케이블) 를 봐야 한다.

확인: `cargo test -p drlabs-dynamixel` 71개 통과(응답 예산·복구·포기·재개 4개 추가).
**실기 미검증** — 다음 soak 에서 볼 것은 (a) `silent on the broadcast` 줄의 빈도와
`recovered`/`lost` 비율, (b) comm_loss e-stop 0, (c) 데모 로그의 팔 오차 스파이크 소멸.
20 ms 예산이 이 호스트의 USB 지연에 비해 짧으면 `recovered` 없이 `lost` 만 찍히다
다음 사이클에 정상으로 돌아오는 패턴이 보일 것이고, 그때는 `reply_timeout` 을 올린다.

### 실기 soak (2026-09-05, 새 바이너리, ubp15.json 10 Hz, `demo_circle.py --shared` 무한)

13분 10초 동안 broadcast 침묵 20회, **전부 개별 재읽기로 복구**, lost 0, comm_loss 0, 팔
오차 ≤ 0.18 rad. 침묵 패턴은 청크 꼬리(5~8, 3~6)와 청크 전체(1~8) 둘 다 있었고, 읽기
단계 소요는 24~35 ms(예산 20 ms + 개별 읽기 1~2 ms × n). 즉 꼬리 유실 → 정지 → e-stop
고리는 끊겼다.

t = 790 s 에 **다른 종류의 고장**: 15축 전부가 broadcast 에도 개별 읽기에도 4사이클 연속
(사이클당 365 ms, 약 1.5 s) 침묵 → 15축 모두 comm_loss e-stop → 데모는 `/api/pose` 409 로
종료(트레이스백; HTTPError 를 안 잡던 데모 결함, 같이 고침). 패널을 내리고 `dxl_diag dump`
전 id ERR, pyserial 로 직접 ping 해도 모든 id 가 0x00 한 바이트만 돌려줌(회선 무송신 시
잡음 없음). COM7(FTDI FT232H) 은 Windows 에서 정상. 직전 텔레메트리에서 우측 서보 전압
11.3~11.4 V, 좌측 12.4~12.5 V(하한 11.0 V 근처). 지난 9-05 00:12 e-stop 도 12분대였다.
버스가 통째로 죽고 돌아오지 않는 것은 코드가 아니라 **전원(PSU 보호 동작·연결부) 또는
서보 하나가 DATA 를 잡고 있는 경우**로 추정 — 전원 재투입 후 `dxl_diag dump` 로 wdog(98)·
p_vel(112) 가 0 이면 서보가 재부팅된 것(전원), 값이 남아 있으면 회선·어댑터 쪽이다.

## 세 번째 전 버스 사망과 자동 복구 — 모션 쇼 데모 (2026-09-05)

원 그리기 soak 세 번째 실행(09:01 시작, `ubp15.json` 10 Hz, `demo_circle.py --shared`)
이 t = 1014 s 에 같은 패턴으로 끝났다: 15축 전부 broadcast·개별 읽기 모두 4사이클
침묵 → 전축 comm_loss e-stop. 그 전 1014 초 동안 침묵 45회는 전부 recovered, lost 0.
직전 우측 전압 11.4~11.6 V(좌 12.4~12.8 V). 세 번 모두 12~17분대.

이번엔 panel 을 내리고 `dxl_diag dump` 를 바로 찍었다: **15축 전부 응답**, wdog(98)
0, p_vel(112) 0, torque 0, present 는 2048 부근(팔이 늘어진 자세). 즉 서보가 전부
재부팅됐고(전원 순간 강하로 확정에 가까움, 어댑터·회선 문제 아님) 버스는 전원 재투입
없이 스스로 돌아왔다. 어제 pyserial ping 이 0x00 만 받은 것은 아직 설명 못 함.

이 고장은 코드로는 못 막지만, 데모가 살아남게는 할 수 있다. `tools/demo_show.py` 는
`--launch configs/ubp15.json` 으로 panel 을 자식 프로세스로 띄우고, 관절이 e-stop
되면 panel 을 끝내고 다시 띄운 뒤(5 → 60 s 백오프) base 로 돌아가 끊긴 act 를 처음부터
한다. `/api/reset` 만으로는 안 되는 이유: 통신 유실 후 세션이 링크를 닫아 버리고
(`control_loop.rs` "Reconnecting is the operator's move") 재연결 API 가 없다. reset 은
통과하지만 15축이 `blocked: ["Not connected."]` 로 남는다(확인함).

같은 날 벤치에서 확인한 것 두 가지:

- **`/api/stop` 은 토크를 풀지 않는다.** panel 의 bring-up 이 auto-arm 이라 disarm
  된 관절이 다음 사이클에 다시 arm 된다(stop 2 s 뒤 15축 armed, 서보 torque 1). 데모가
  기계를 놓을 때는 `/api/estop` 을 쓴다: latch 되어 재arm 이 없고 stop burst 가 토크
  off 를 실어 나간다(종료 후 dump 에서 torque 0, wdog 20 확인). panel 자체는 Ctrl+C
  핸들러가 없어 그냥 죽으므로, 스크립트가 estop → 1 s → 프로세스 종료 순서를 지킨다.
- **`wait_settled` 는 프로파일 이동 중에 바로 돌아온다.** `target_rad` 가 프로파일을
  따라가므로 위치 오차가 처음부터 작다. 첫 setpoint 에서 팔이 0.76 rad 뒤처진 채로
  스트리밍이 시작됐다. `demo_show.py` 의 `approach` 는 `moving_to_rad` 가 null 이 되고
  오차가 허용치 안일 때를 도착으로 본다. `demo_circle.py` 는 손대지 않았다(원의 첫
  점이 base 와 가까워 실제로는 문제가 안 됐다).

쇼 실기(1사이클, 8자 3회 12 s + 손 흔들기 12 s + 휴식 2 s): 8자 팔 오차 ≤ 0.12 rad,
손 흔들기(sh_p -0.75, elb_p -1.4, elb_y ±0.4 rad 0.5 Hz) 처짐 0.33 rad, 우측 전압
최저 10.8 V(MX 하한 10.0). 강제 `/api/estop` 으로 복구 경로 확인: 5 s 뒤 panel 재기동,
15축 armed, base 복귀, act 재시작까지 9 s.

## 버스 단위 드라이버와 Fast Sync Read (2026-10-27)

`DynamixelBus`(`src/bus.rs`)는 `Chain`의 반대 모양이다. `Chain`은 조인트 스레드 N개가
포트 하나를 두고 "먼저 잡은 쪽이 버스 전체 트랜잭션"을 하는 구조였고, `DynamixelBus`는
스레드 하나가 축 전부를 한 틱에 처리한다. 뮤텍스를 기다릴 상대가 없다.

한 틱의 와이어 순서: 토크 off(이탈 축) → 바뀐 한계(watchdog 546/98, accel, velocity —
바뀐 것만) → 토크 on(진입 축) → goal position sync write 1회 → present block 읽기 →
1초마다 hardware error·bus watchdog 레지스터. 정상 상태에서는 **쓰기 1 + 읽기 1**.
틱 안에서 EEPROM 쓰기·개별 복구 읽기·응답 예산 이상 대기는 하지 않는다(침묵 축은
`Silent`로 보고하고 다음 틱에 다시 묻는다). 운영 모드는 open 때 Extended Position으로
맞춰 두고 틱에서는 바꾸지 않는다.

**Fast Sync Read (0x8A)** — 확인함(ROBOTIS Protocol 2.0 문서, 공식 SDK 수신 코드):
- 요청은 Sync Read와 같은 파라미터, 응답은 **패킷 1개**(ID 0xFE). 첫 서보가 시작하고
  이후 서보가 `ERR, ID, data, CRC`를 같은 스트림에 덧붙인다. 세그먼트 stride는
  `X+4`, 전체 길이는 `8 + n(X+4)` 바이트.
- 공식 SDK(`protocol2_packet_handler.cpp`)는 **마지막 CRC만** 전체 패킷에 대해 검사하고
  (`updateCRC(0, rxpacket, wait_length-2)`), 중간 CRC는 stride로 건너뛰며,
  byte stuffing 제거를 하지 않는다(`skip_stuffing`). 이 드라이버도 같다.
- 지원 여부는 펌웨어에 달렸다. 미지원 서보는 자기 ID로 Instruction Error(0x02)를
  단독 응답하고, 드라이버는 `FastOutcome::Unsupported(id)`로 보고한다. open 때 서보마다
  단독으로 한 번 보내 지원군/미지원군을 나누고, 미지원군은 기존 Sync Read 8개 청크로
  같은 틱에 읽는다. 지원 펌웨어 버전은 **미확인**(3rd-party 문서: X430/X540 v45+,
  X330 v47+; MX(2.0)은 모름).

와이어 예산(계산값, 미측정): 15축 MX/X, 3 Mbps, 10 바이트 블록 —
Fast Sync Read 전부 지원 시 29+218+89 = 336 B ≈ 1.12 ms/틱(56 %) + RDT 15×20 µs.
현재 청크 방식은 ≈ 441 B + USB 왕복 2회. 26축 P시리즈 1버스는 556 B → 3 M에서 93 %,
4 M에서 70 %; 2버스 × 13이면 3 M에서 49 %.

측정 도구(`examples/dxl_diag.rs`):
```
dxl_diag COM7 3000000 fastprobe 1,2,...,16          # 서보별 모델·펌웨어·0x8A 수락 여부
dxl_diag COM7 3000000 syncstress <ids> 8 20 hz=500 write        # 현재 방식
dxl_diag COM7 3000000 syncstress <ids> 8 20 hz=500 write fast   # Fast Sync Read
```
`fastprobe`가 설계를 가른다. 측정값은 이 아래에 적는다 — 2026-10-27 현재 COM7 미연결로
**미측정**.

P시리즈 프로파일 `ph54`/`pm54`는 e-Manual에서 확인한 주소(574/576/580/592/594/564/550/546/9)
외에는 기억에 의존한 **미검증** 표이며, 모델 번호로는 선택되지 않고 이름으로만 선택된다.

## macOS: IOSSIOSPEED, 그리고 아직 재지 않은 FTDI 레이턴시 타이머 (2026-10-29)

Apple Silicon Mac에서 처음으로 Rust 트리를 빌드하고 돌렸다. 하드웨어는 아직
Mac에 연결하지 않았으므로 이 절은 **구현한 것과 측정해야 할 것**을 적는다.
수치는 벤치에서 채운다.

**구현.** macOS는 비표준 보레이트를 `cfsetspeed`로 받지 않는다. 1 000 000을
넣으면 성공을 돌려주고 어댑터는 이전 속도에 머물러, 체인의 서보 전부가 침묵한다.
`crates/drlabs-transport/src/serial/unix.rs`는 이제 `tcsetattr` 뒤에
`IOSSIOSPEED` ioctl(`_IOW('T', 2, speed_t)` = 0x80085402)로 속도를 건다.
Apple의 내장 FTDI/CDC 드라이버가 이 호출을 받는다. 같은 파일의 `write`는
non-blocking fd의 부분 쓰기를 끝까지 채우고, 200 ms 동안 빠지지 않으면
TimedOut — sync write 한 패킷의 절반만 선에 실리면 모든 서보에서 CRC 오류다.

**측정할 것 — 레이턴시 타이머.** FTDI 칩은 수신 버퍼가 62바이트로 차거나
레이턴시 타이머가 만료될 때만 호스트로 올린다. Linux `ftdi_sio`는 이 타이머를
1 ms로 둔다. Apple 내장 드라이버의 기본값은 16 ms로 알려져 있고, 그대로라면
Dynamixel 상태 패킷(수십 바이트)마다 최대 16 ms를 기다려 라운드트립이 60 Hz
근처에 묶인다 — [위 절](#the-serial-read-timeout-was-the-loop-rate)의 Windows
10 ms 타임아웃과 정확히 같은 모양의, "성공을 보고하는 느린 루프"다.
`IOSSDATALAT` ioctl(`_IOW('T', 3, unsigned long)`, µs 단위)이 이를 낮추는
공식 경로이나 Apple 드라이버가 받는지는 재 봐야 안다.

절차(U2D2 + XL330-M288-T, 1 Mbaud):

```
drlabs ports                                            # /dev/cu.usbserial-XXXX 확인
drlabs probe -driver dynamixel -port /dev/cu.usbserial-XXXX -baud 1000000 -id 1
                                                        # 식별되면 IOSSIOSPEED 증명
cargo run --release -p drlabs-dynamixel --example dxl_diag -- \
    /dev/cu.usbserial-XXXX 1000000 syncstress 1 8 20 hz=500 write fast
                                                        # 라운드트립과 유지 레이트
```

비교 기준은 Windows COM7 3 Mbaud 15축의 `syncstress` 수치(위 2026-10-27 절)와
Windows COM6 1 Mbaud 단축 1.12 ms/cycle. 라운드트립이 15 ms 근처면 레이턴시
타이머다: `IOSSDATALAT`로 1000 µs를 걸어 다시 재고, 드라이버가 거부하면 FTDI의
D2XX 대신 `IOSSDATALAT`을 받는 서드파티 VCP 드라이버가 유일한 길이다.

| | per cycle | rate | 비고 |
|---|---|---|---|
| macOS, open 직후 | 미측정 | 미측정 | |
| macOS, IOSSDATALAT 1 ms | 미측정 | 미측정 | 드라이버 수락 여부 포함 |

fdcanusb(CDC-ACM)는 레이턴시 타이머가 없어 이 문제가 없다. 다만 STM32 CDC
스택이 DTR을 요구하는 경우가 있어, `probe -driver moteus`가 Mac에서 침묵하면
open 직후 `TIOCMBIS(TIOCM_DTR|TIOCM_RTS)`를 추가하는 것이 첫 시도다.

