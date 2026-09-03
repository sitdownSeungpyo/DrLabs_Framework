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

## 미해결: sync write goal 이 큰 변화를 서보에 반영하지 못한다 (2026-09-03)

15축 UBP(`configs/ubp.json`, COM7 3Mbaud, id 11 XL430 은 12.3V over-voltage 로 제외한
15축)를 base 포즈로 trajectory 이동(panel `POST /api/move {"pose":"base"}` →
`Robot::move_to_pose` → `start_trajectory`)하면 arming·trajectory 는 정상인데 서보가
목표까지 가지 않는다. 층별로 좁힌 결과, 원인은 **session 도 trajectory 도 아니고**
sync write goal 이 서보에 반영되지 않는 실물 동작 하나로 좁혀졌으나 그 이유는 미확정.

확인한 사실(모두 벤치 실측):

- **session / trajectory / setpoint 는 전부 정상.** setpoint → `command.position` →
  goal counts 가 한 필드(`state.setpoint.position`)에서 나오고, trajectory 가
  progress 0→1 로 정상 진행하며, chain 이 write 하려는 goal counts = setpoint
  (확인함: write=2219 = base target 0.262 rad). 이 경로 전체를 전수 확인했다.
- **문제는 서보에 안 먹히는 것.** chain 의 sync write(0x83, broadcast)로 goal 2219 를
  보내도 서보 Goal Position(116)은 2050(현재)에 머문다 — write 직후 같은 Driver 로
  reg 116 을 read 해 확인(`wrote=2219 servo_reg116=2050`). 진행 중의 작은 변화(2053)는
  반영되나 큰 변화(2219)는 안 된다.
- **개별 write(0x03)로 바꾸면** 값은 서보에 반영되나, 15축에서는 그 status 응답이
  chain 의 sync read 응답 스트림과 섞여 0x04 data-range-error 경합 → link_error e-stop.
- **run(단일 서보, 개별 write)은 큰 goal 도 움직인다** (0.499→0.616 rev 확인) — 경합이
  없어서. 서보·드라이버·개별 write 자체는 정상.
- **Drive Mode(reg 10) = 0**: 전원 재투입 후 DYNAMIXEL Wizard 로 전 서보 확인,
  velocity-based 이며 Time-based Profile 아님. Time-based 리셋 가설은 **기각**.
- 주의: 디버깅 중 서보 Status Return Level(reg 68, RAM)을 1로 바꿨다가 전원 재투입으로
  복원함. 다음에 무응답이면 이 값(2 여야)부터 의심.

다음 단계(우선순위):

1. Wizard 에서 Goal Position 을 **큰 값으로 직접 write** 했을 때 서보가 움직이는지 —
   움직이면 sync write broadcast 패킷 문제(15서보 79바이트 params 의 파싱/정렬/CRC),
   안 움직이면 서보 Profile Configuration/Drive Mode 이외의 설정 문제.
2. 같은 goal 값에 대해 sync write(0x83) 패킷과 개별 write(0x03) 패킷을 byte 로 비교.
3. 유력한 수정 방향: goal 을 chain 의 단일 스레드가 **순차 개별 write(응답 받음)**로
   방출 — sync read 와 안 섞이므로 경합 없이 개별 write 의 이점(반영됨)을 얻는다.
   이번에 시도했을 때 e-stop 이 났으나, 그때는 별개로 setpoint 가 아직 target 이 아닌
   시점이었을 가능성이 있어(진행 중 값과 완료 값이 섞임) 재검증이 필요하다.

이번 세션에 완성·커밋된 것(별개, 정상 동작): 제어 주기 hot-path 최적화, 그리고
단일 I/O 아키텍처로 15축 100Hz arming 달성. base **이동**만 위 이유로 미완이다.
