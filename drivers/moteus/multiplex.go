package moteus

import (
	"encoding/binary"
	"fmt"
	"math"
)

// The mjlib multiplex register protocol, as used by moteus over CAN-FD.
//
// A frame is a sequence of subframes.  Each begins with a command byte whose
// upper nybble is the operation and whose low bits carry the value type and,
// for short runs, the count:
//
//	0x00 | type<<2 | n   write n registers of that type
//	0x10 | type<<2 | n   read n registers
//	0x20 | type<<2 | n   reply with n register values
//	0x30 / 0x31          write error / read error
//	0x40 .. 0x44         tunnelled diagnostic stream
//	0x50                 no-op (also used as frame padding)
//
// When n is 0 the count follows as a varuint, which is how runs longer than
// three registers are expressed.  The starting register number follows as a
// varuint, and subsequent values apply to consecutive registers.
//
// Reference: https://github.com/mjbots/moteus/blob/main/docs/reference.md

// Resolution is the on-wire width of a register value.  Narrower is not just
// smaller: a command and its telemetry query share one 64-byte frame, so
// resolution is the budget that decides how much can be asked for per cycle.
type Resolution uint8

const (
	Int8  Resolution = 0
	Int16 Resolution = 1
	Int32 Resolution = 2
	F32   Resolution = 3
	// Ignore marks a field that should not be transmitted at all.
	Ignore Resolution = 4
)

// resolutionUnset is the sentinel the write combiner starts from, so that the
// first register always emits a framing byte.
const resolutionUnset Resolution = 0xff

func (r Resolution) String() string {
	switch r {
	case Int8:
		return "int8"
	case Int16:
		return "int16"
	case Int32:
		return "int32"
	case F32:
		return "f32"
	case Ignore:
		return "ignore"
	}
	return "invalid"
}

// Size is the number of bytes a value of this resolution occupies.
func (r Resolution) Size() int {
	switch r {
	case Int8:
		return 1
	case Int16:
		return 2
	case Int32, F32:
		return 4
	}
	return 0
}

// Subframe operation bases.
const (
	writeBase  = 0x00
	readBase   = 0x10
	replyBase  = 0x20
	writeError = 0x30
	readError  = 0x31
	nop        = 0x50
)

// Per-resolution write command offsets, indexed by Resolution.
var writeCommands = [4]byte{0x00, 0x04, 0x08, 0x0c}

// typeMax is the largest magnitude each integer resolution can carry.  The
// negative-most value is reserved as NaN, so saturation stops one short of it.
var typeMax = [4]float64{127, 32767, 2147483647, math.NaN()}

// writer accumulates the bytes of a multiplex frame.
type writer struct {
	buf []byte
}

func (w *writer) size() int     { return len(w.buf) }
func (w *writer) bytes() []byte { return w.buf }

func (w *writer) writeU8(v byte) { w.buf = append(w.buf, v) }

func (w *writer) writeInt8(v int8) { w.buf = append(w.buf, byte(v)) }

func (w *writer) writeInt16(v int16) {
	w.buf = binary.LittleEndian.AppendUint16(w.buf, uint16(v))
}

func (w *writer) writeInt32(v int32) {
	w.buf = binary.LittleEndian.AppendUint32(w.buf, uint32(v))
}

func (w *writer) writeF32(v float32) {
	w.buf = binary.LittleEndian.AppendUint32(w.buf, math.Float32bits(v))
}

// writeVaruint emits an unsigned integer seven bits at a time, most
// significant continuation bit set on every byte but the last.
func (w *writer) writeVaruint(v uint32) {
	for {
		b := byte(v & 0x7f)
		v >>= 7
		if v != 0 {
			b |= 0x80
		}
		w.buf = append(w.buf, b)
		if v == 0 {
			return
		}
	}
}

// writeValue emits value already converted to the raw units of res.
func (w *writer) writeValue(value float64, res Resolution) {
	switch res {
	case Int8:
		w.writeInt8(int8(value))
	case Int16:
		w.writeInt16(int16(value))
	case Int32:
		w.writeInt32(int32(value))
	case F32:
		w.writeF32(float32(value))
	}
}

// saturate converts a physical value into the raw integer the wire carries.
//
// A non-finite input becomes the negative-most representable value, which is
// this protocol's NaN: for a command register that means "no opinion, use
// whatever the controller already has", which is exactly what `d pos … nan`
// does in tview.
func saturate(value float64, res Resolution, scale float64) float64 {
	if math.IsNaN(value) || math.IsInf(value, 0) {
		switch res {
		case Int8:
			return -128
		case Int16:
			return -32768
		case Int32:
			return -2147483648
		default:
			return value
		}
	}

	scaled := value / scale
	if res == F32 {
		return scaled
	}

	max := typeMax[res]
	if scaled < -max {
		return -max
	}
	if scaled > max {
		return max
	}
	// Truncate towards zero, matching the reference implementation's int().
	return math.Trunc(scaled)
}

// writeMapped emits a physical value using the per-resolution scale factors.
func (w *writer) writeMapped(value float64, s8, s16, s32 float64, res Resolution) {
	scale := scaleFor(res, s8, s16, s32)
	w.writeValue(saturate(value, res, scale), res)
}

func scaleFor(res Resolution, s8, s16, s32 float64) float64 {
	switch res {
	case Int8:
		return s8
	case Int16:
		return s16
	case Int32:
		return s32
	}
	return 1.0
}

// combiner groups consecutive registers of the same resolution into a single
// subframe.  Emitting one framing byte plus one register number for a run of
// four is the difference between a command that fits alongside its telemetry
// query in one frame and one that does not.
type combiner struct {
	w             *writer
	baseCommand   byte
	startRegister uint32
	resolutions   []Resolution

	// replySize accumulates how many bytes the controller's answer will
	// occupy, which lets a caller check a query against the frame budget
	// before sending it.
	replySize int

	offset  int
	current Resolution
}

func newCombiner(w *writer, baseCommand byte, startRegister uint32, resolutions []Resolution) *combiner {
	return &combiner{
		w:             w,
		baseCommand:   baseCommand,
		startRegister: startRegister,
		resolutions:   resolutions,
		current:       resolutionUnset,
	}
}

func (c *combiner) size() int { return len(c.resolutions) }

// maybeWrite advances one register and reports whether the caller should now
// emit its value.  It writes framing bytes whenever the resolution changes.
func (c *combiner) maybeWrite() bool {
	if c.offset >= len(c.resolutions) {
		return false
	}
	thisOffset := c.offset
	c.offset++

	if c.current == c.resolutions[thisOffset] {
		// Still inside the current run: no framing needed, and the value goes
		// out only if this run is not a run of ignores.
		return c.current != Ignore
	}

	next := c.resolutions[thisOffset]
	c.current = next
	if next == Ignore {
		return false
	}

	// Look ahead to see how far the new resolution extends.
	count := 1
	for i := thisOffset + 1; i < len(c.resolutions) && c.resolutions[i] == next; i++ {
		count++
	}

	command := writeCommands[next] | c.baseCommand
	startSize := c.w.size()

	if count <= 3 {
		c.w.writeU8(command + byte(count))
	} else {
		c.w.writeU8(command)
		c.w.writeU8(byte(count))
	}
	c.w.writeVaruint(c.startRegister + uint32(thisOffset))

	c.replySize += c.w.size() - startSize
	c.replySize += count * next.Size()
	return true
}

// writeAll runs the combiner to completion without emitting values.  It is
// what a read request needs: a read subframe names registers but carries no
// data.
func (c *combiner) writeAll() {
	for i := 0; i < c.size(); i++ {
		c.maybeWrite()
	}
}

// regValue is one decoded register from a reply, still in raw protocol units.
type regValue struct {
	res Resolution
	// raw is the wire value with the protocol's NaN sentinel already turned
	// into a real NaN, but before the register's scale factor is applied.
	raw float64
}

// reader walks the bytes of a received frame.
type reader struct {
	data   []byte
	offset int
}

func (r *reader) remaining() int { return len(r.data) - r.offset }

func (r *reader) readU8() (byte, error) {
	if r.remaining() < 1 {
		return 0, errShort
	}
	v := r.data[r.offset]
	r.offset++
	return v, nil
}

func (r *reader) readVaruint() (uint32, error) {
	var result uint32
	var shift uint
	for i := 0; i < 5; i++ {
		if r.remaining() < 1 {
			return 0, errShort
		}
		b := r.data[r.offset]
		r.offset++
		result |= uint32(b&0x7f) << shift
		shift += 7
		if b&0x80 == 0 {
			return result, nil
		}
	}
	return 0, fmt.Errorf("moteus: varuint longer than 5 bytes")
}

func (r *reader) readValue(res Resolution) (float64, error) {
	size := res.Size()
	if size == 0 || r.remaining() < size {
		return 0, errShort
	}
	chunk := r.data[r.offset : r.offset+size]
	r.offset += size

	switch res {
	case Int8:
		return nanify(float64(int8(chunk[0])), res), nil
	case Int16:
		return nanify(float64(int16(binary.LittleEndian.Uint16(chunk))), res), nil
	case Int32:
		return nanify(float64(int32(binary.LittleEndian.Uint32(chunk))), res), nil
	case F32:
		return float64(math.Float32frombits(binary.LittleEndian.Uint32(chunk))), nil
	}
	return 0, fmt.Errorf("moteus: unknown resolution %d", res)
}

var errShort = fmt.Errorf("moteus: truncated multiplex frame")

// nanify turns the protocol's negative-most sentinel into a real NaN, so that
// "the controller has no value for this" survives into the decoded telemetry
// instead of appearing as a large negative measurement.
func nanify(value float64, res Resolution) float64 {
	switch res {
	case Int8:
		if value == -128 {
			return math.NaN()
		}
	case Int16:
		if value == -32768 {
			return math.NaN()
		}
	case Int32:
		if value == -2147483648 {
			return math.NaN()
		}
	}
	return value
}

// parseReply decodes the reply subframes of a received frame into register
// values.  Read and write subframes are skipped: a reply frame should not
// contain them, but a bus shared with other traffic can deliver anything.
//
// A truncated frame yields whatever was decoded before the break together
// with the error, because a partial reply is still worth reporting — losing
// the tail of a telemetry frame should not blind the safety monitor to the
// mode and fault registers at its head.
func parseReply(data []byte) (map[Register]regValue, error) {
	values := make(map[Register]regValue, 16)
	r := &reader{data: data}

	for r.remaining() > 0 {
		cmd, err := r.readU8()
		if err != nil {
			return values, err
		}

		upper := cmd & 0xf0
		res := Resolution((cmd & 0x0c) >> 2)
		shortCount := uint32(cmd & 0x03)

		switch {
		case upper == replyBase || upper == writeBase || upper == readBase:
			count := shortCount
			if count == 0 {
				if count, err = r.readVaruint(); err != nil {
					return values, err
				}
			}
			if count == 0 {
				continue
			}
			start, err := r.readVaruint()
			if err != nil {
				return values, err
			}
			for i := uint32(0); i < count; i++ {
				if upper == readBase {
					// A read subframe names registers but carries no values.
					continue
				}
				value, err := r.readValue(res)
				if err != nil {
					return values, err
				}
				if upper == replyBase {
					values[Register(start+i)] = regValue{res: res, raw: value}
				}
			}

		case cmd == writeError || cmd == readError:
			register, err := r.readVaruint()
			if err != nil {
				return values, err
			}
			code, err := r.readVaruint()
			if err != nil {
				return values, err
			}
			return values, fmt.Errorf("moteus: controller rejected register 0x%03x with error %d", register, code)

		case cmd == nop:
			continue

		default:
			// Anything else means the stream is not what we think it is;
			// decoding further would invent register values.
			return values, fmt.Errorf("moteus: unexpected subframe command 0x%02x", cmd)
		}
	}
	return values, nil
}
