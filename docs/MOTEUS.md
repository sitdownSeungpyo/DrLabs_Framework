# moteus protocol notes

Working notes on the parts of the moteus protocol this framework implements,
and on the details that cost time to get right. The authority is the
[moteus reference documentation](https://github.com/mjbots/moteus/blob/main/docs/reference.md);
this file records what the implementation depends on and why.

## Frame layout

A moteus exchange is one CAN-FD frame out and one back.

### Arbitration ID

```
bits  0- 7   destination controller id
bit      15  set when a reply is required
bits  8-14   source id (the host, usually 0)
bits 16-28   optional bus prefix, for segmenting one physical bus
```

A frame with the reply bit set produces an answer addressed back to the source.
Replies carry the *controller's* id in the source field, which is how a shared
bus is demultiplexed — `Controller.exchange` discards anything whose source
does not match.

### Payload: the multiplex register protocol

The payload is a sequence of subframes. Each begins with a command byte:

| Byte | Meaning |
|------|---------|
| `0x00 \| type<<2 \| n` | write `n` registers of that type |
| `0x10 \| type<<2 \| n` | read `n` registers |
| `0x20 \| type<<2 \| n` | reply with `n` register values |
| `0x30` / `0x31` | write error / read error |
| `0x40`–`0x44` | tunnelled diagnostic stream |
| `0x50` | no-op, also used as frame padding |

`type` is 0 for int8, 1 for int16, 2 for int32, 3 for float32. When `n` is 0
the count follows as a varuint, which is how runs longer than three registers
are expressed. The starting register number follows as a varuint; subsequent
values apply to consecutive registers.

A varuint is seven bits per byte, least significant first, with the top bit set
on every byte but the last.

### Value scaling

Integer registers are fixed-point. The scale depends on both the register
family and the resolution it was sent at:

| Family | int8 | int16 | int32 | Unit |
|--------|------|-------|-------|------|
| position | 0.01 | 0.0001 | 0.00001 | rev |
| velocity | 0.1 | 0.00025 | 0.00001 | rev/s |
| acceleration | 0.05 | 0.001 | 0.00001 | rev/s² |
| torque | 0.5 | 0.01 | 0.001 | Nm |
| pwm / scale | 1/127 | 1/32767 | 1/2147483647 | — |
| voltage | 0.5 | 0.1 | 0.001 | V |
| temperature | 1.0 | 0.1 | 0.001 | °C |
| time | 0.01 | 0.001 | 0.000001 | s |
| current | 1.0 | 0.1 | 0.001 | A |
| power | 10.0 | 0.05 | 0.0001 | W |

Float32 registers carry the physical value directly.

**Truncation, not rounding.** `0.25 / 0.00001` is 24999.999… in binary floating
point, and the protocol truncates towards zero, so the wire carries 24999.
Rounding instead would put nearly every position command one count away from
what the reference implementation produces. `saturate` in `multiplex.go` does
this deliberately, and `TestSaturateTruncatesTowardsZero` pins it.

**The NaN sentinel.** The negative-most value of each integer width is NaN:
`-128`, `-32768`, `-2147483648`. Saturation therefore clamps to `±(max)`, one
short of the negative end, so a large negative value cannot be mistaken for
"no value".

## Two meanings of NaN, and why the code separates them

This is the detail most likely to produce a bug that looks like a mechanical
fault.

On this protocol, NaN in a *command* register is meaningful:

- **Position register.** NaN means "no position target". It is how velocity
  mode is expressed: the axis holds the commanded velocity instead of servoing
  anywhere.
- **Maximum torque register.** NaN means "use the controller's own configured
  limit" — what `d pos … nan` does in tview.

Omitting a register is a *different* thing. Command registers are sticky: a
value written stays in force until it is written again. Not sending a register
leaves whatever was last put in it.

So "send NaN" and "do not send" cannot share a representation. If they did, a
velocity command would omit the position register, the controller would keep
the previous position target, and the axis would servo back to it instead of
running at the commanded velocity — a motor that appears to fight its own
command for no visible reason.

`frames.go` therefore carries an `opt` type with the value and a `send` flag,
and `motor.Command` has an explicit `DeferTorqueToController` flag rather than
overloading a NaN in `MaxTorque`.

The same trap appears in the safety layer: `min`/`max` against a NaN silently
collapses to the bound, so clamping a deferred torque limit would turn "use the
controller's limit" into "command zero torque". `motor.Clamp` returns NaN
unchanged, and `ClampCommand` skips the torque clamp entirely when the limit
has been deferred.

## Bandwidth is the binding constraint

A CAN-FD frame carries at most 64 bytes, and a command shares that frame with
the telemetry query it piggybacks on. Asking for everything at full precision
every period is what stops a loop from reaching its target rate.

Three query profiles rotate:

| Profile | Contents | Reply | Sent |
|---------|----------|-------|------|
| fast | mode, position, velocity, torque, q current, voltage, board temperature, fault | ~28 B | every cycle |
| full | fast + d current, absolute position, power, motor temperature, trajectory complete, home state | ~40 B | every 10th |
| verify | fast + the controller's *active* command registers | ~40 B | every 97th |

The verify divisor is prime so that verify and full cycles do not repeatedly
coincide — two wide queries landing on the same period is exactly the frame
that overruns. And a full parameter frame is never sent together with a wide
query; `profileForLocked` demotes the profile to fast on those cycles.

The command side is split the same way. Because command registers are sticky,
the per-cycle frame only carries mode, position, velocity, maximum torque and
the watchdog timeout — about 21 bytes. The rest goes out only when the operator
changes something, and is re-sent every two seconds so a controller that reset
underneath us does not keep running on limits nobody chose.

### Why verify exists

The verify profile reads back the controller's live command registers and
compares them with what the host believes it set. Command registers survive
things the host does not see: a brownout that resets the controller, a dropped
frame, another program on the same bus writing them. When they disagree, the
parameter frame is re-sent immediately.

This turns "the machine behaved unexpectedly" into "the controller's torque
limit is 0.05 Nm, not the 0.4 we set" — a sentence someone can act on.

## Identifying the board

The family and hardware revision have no register. They exist only in the
`firmware` telemetry record, which is normally emitted as a binary frame — and
decoding that would need the schema parser this driver deliberately does
without.

The console has a way out. Asking it to emit the channel as text turns the
record into six readable lines:

```
tel fmt firmware 1
tel get firmware
  firmware.version 270
  firmware.model 0
  firmware.family 0
  firmware.hwrev 8
```

`Identify` does exactly that, and `BoardStatus(0, 8)` then resolves to a
supported moteus r4.11. Before this, every moteus connection reported an
unidentified board and had to be waved through by hand — which is worse than it
sounds, because an override that is needed every single time stops being a
decision and becomes a reflex, including on the boards where it should not be.

Switching the format is a runtime setting on an emission that is already
stopped, so it disturbs nothing else on the bus.

## The travel envelope, and fault 39

The controller stores its own travel limits in `servopos.position_min/max`.
They are configuration, not state, so they are only reachable through the
diagnostic console — which is why this framework implements the tunnelled
stream at all.

They matter out of proportion to their obscurity:

- Starting position control while the axis is **outside** the bounds raises
  fault 39 ("outside limit") immediately.
- Commanding a target **past** the bounds makes the firmware clamp it and
  report code 103 ("limiting: position bounds").

From the outside both look exactly like "the motor ignores me". Reading the
bounds at connect time lets the preflight check say what is actually wrong and
name the three ways out: turn the shaft back inside by hand (the drive is off,
so it is free), re-reference the output while disarmed, or set
`IgnorePositionBounds` on the command.

The session also keeps commands `boundsInset` (0.05 rev) inside the firmware
bound. Commanding exactly the limit leaves no room for overshoot, and the
firmware faults the moment position control starts at or beyond it.

## The console has no way to match a reply to its request

`conf get servopos.position_min` answers with `-5`. Not `servopos.position_min
-5` — just the value. There is nothing in the reply that says which question it
answers, so a console read is always a read of "whatever the controller says
next", and the only thing making that the right answer is that the console was
quiet beforehand.

It is easy for it not to be. A command that produces more output than the
caller reads leaves the controller emitting into the same channel: `conf
enumerate` is thousands of lines, and a read that times out half way through
anything leaves the rest queued. Every read after that is off by some number of
lines, and reports another key's value as this key's value.

Measured on an r4.11 before this was addressed: after one unread `conf
enumerate`, `servopos.position_min` came back as `motor.cogging_dq_comp.422 0`,
and `Identify` reported `board="unknown"`, `bounds=[NaN, NaN]` — while
returning a nil error. That is the state this file's "Identifying the board"
section exists to prevent, arrived at silently.

So `Query` and `Command` wait for the console to go quiet before writing, and
return `ErrConsoleBusy` if it will not. Refusing is the whole point: a travel
bound invented from a stale line is worse than an absent one, because the
preflight check would then trust it. `Identify` stops after its second drain
for the same reason, and reports an unidentified board rather than a
fabricated envelope.

`Drain` reports whether it succeeded. One call is bounded — a real backlog
outlasts it, measured at 16.7 s over three rounds — so a caller that needs the
console back calls it until it returns true.

The cost is at connect and nowhere else: `Identify` went from 313 ms to
472 ms, and the control loop is untouched at 200.0 Hz because the console is
never read inside it.

## Fault codes

Codes below 96 are latching faults. Codes from 96 up are *limiting* states:
the controller is capping its output and saying by what. They are not faults
and must not stop the machine — a correctly torque-limited axis would be
unusable.

Code 102, "limiting: max torque", deserves special mention: it is the most
common reason a moteus sits still while cheerfully accepting commands. Surfacing
it as a warning with its explanation is worth more than any amount of generic
error handling.

## The fdcanusb line protocol

```
host -> device   can send <id_hex> <data_hex> [flags]\n
device -> host   OK\n
device -> host   rcv <id_hex> <data_hex> [flags]\n
device -> host   ERR <text>\n
```

Flags are concatenated letters: `B` bit-rate switch on, `b` off, `F` CAN-FD,
`E` extended identifier.

Points that matter in practice:

- **Wait for the `OK`.** Without it a caller can queue frames faster than the
  adapter drains them, and the backlog surfaces much later as telemetry
  belonging to a command several cycles old.
- **A reply can overtake its own acknowledgement.** `Send` keeps any `rcv` line
  that arrives before the `OK` and hands it to the next `Recv`.
- **Padding is `0x50`.** CAN-FD has no byte-granular length above 8, so a
  payload is padded up to the next valid DLC (8, 12, 16, 20, 24, 32, 48, 64).
  `0x50` is the multiplex NOP, so the controller skips the padding instead of
  reading it as register operations.
- **Checksums are optional and sticky.** If the adapter rejects a command with
  an error mentioning "checksum", every subsequent line in both directions
  carries a trailing ` *XX` CRC-8 (polynomial 0x97, nybble table). Once on, an
  incoming line without one is not trustworthy and is discarded.

## Two things that only showed up on hardware

**The sticky registers have to actually be sticky.** The command side is
described above as carrying only mode, position, velocity, maximum torque and
the watchdog per cycle. The driver did not implement that at first — it sent
every populated field every cycle, about 48 bytes — and on the one cycle in ten
where a full telemetry query landed on top, the frame came to 66 bytes. The
adapter refuses an over-long frame, so the symptom was not a truncated reply
but a dead link: `payload of 66 bytes exceeds CAN-FD limit`.

The driver now tracks what the controller was last told and sends a parameter
only when it changed or when a full refresh is asked for. A steady cycle
dropped from 58 bytes to 33. `TestCycleFramesFitInOneCANFDFrame` covers every
combination of refresh and query profile against the 64-byte limit; it fails on
the old code.

The frame budget is now enforced in the driver rather than assumed by the
caller: `fitQuery` measures the command and narrows the query to fit beside it.
Losing a cycle of the wider registers costs one rotation; overflowing costs the
connection.

**The serial read timeout was the loop rate.** The Windows serial port opens
with `ReadTotalTimeoutConstant = 10`, which is right for a Dynamixel packet and
catastrophic for a 200 Hz CAN loop: every read that found an empty buffer cost
ten milliseconds, and the loop ran at 96 Hz however it was configured.

`fdcanusb.Open` now asks the port for a 1 ms read timeout through the optional
`serial.ReadTimeouter` interface, using the Windows combination that returns as
soon as any byte arrives. Measured on the bench afterwards: **200.2 Hz against
a requested 200**, up from 96.

## Transaction timeouts

A moteus transaction must **not** be cancelled because the control period
elapsed. Cancelling mid-flight leaves the adapter's pending-request state
inconsistent, after which every following transaction returns empty — the
reference implementation records a measured case where a 9 ms cap on a 100 Hz
loop turned a 0-in-400 loss rate into 400-in-400.

`Controller.Timeout` is therefore 2 seconds by default. It exists to catch a
wedged adapter, not to enforce the period, and when it fires the link should be
torn down rather than reused. A single missed reply inside the budget is
reported as a dropped cycle (nil telemetry, nil error), not as a failure.
