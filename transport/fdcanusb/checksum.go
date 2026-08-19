package fdcanusb

import (
	"bytes"
	"fmt"
	"strconv"
)

// crc8Table is the nybble-indexed table for CRC-8 with polynomial 0x97, the
// variant the fdcanusb firmware uses for its optional line checksums.
var crc8Table = [16]byte{
	0x00, 0x97, 0xb9, 0x2e, 0xe5, 0x72, 0x5c, 0xcb,
	0x5d, 0xca, 0xe4, 0x73, 0xb8, 0x2f, 0x01, 0x96,
}

// crc8 computes the checksum a nybble at a time.
func crc8(data []byte) byte {
	var crc byte
	for _, b := range data {
		crc = crc8Table[((crc>>4)^(b>>4))&0x0f] ^ (crc << 4)
		crc = crc8Table[((crc>>4)^(b&0x0f))&0x0f] ^ (crc << 4)
	}
	return crc
}

// appendChecksum returns line with " *XX" appended.
//
// The firmware computes the CRC over everything before the '*', which
// includes the separating space — so the space has to be part of the input,
// not just of the output.
func appendChecksum(line []byte) []byte {
	withSpace := append(append([]byte{}, line...), ' ')
	return append(withSpace, []byte(fmt.Sprintf("*%02X", crc8(withSpace)))...)
}

// stripChecksum removes a trailing " *XX" and verifies it.
//
// It returns the line without the checksum, whether one was present at all,
// and whether it matched.  A line with no checksum is reported as valid=false
// but present=false, so the caller can decide whether that is acceptable —
// which depends on whether the adapter has demanded checksum mode.
func stripChecksum(line []byte) (stripped []byte, present bool, valid bool) {
	star := bytes.LastIndexByte(line, '*')
	if star < 0 || star+3 != len(line) {
		return line, false, false
	}

	claimed, err := strconv.ParseUint(string(line[star+1:star+3]), 16, 8)
	if err != nil {
		return line, false, false
	}

	content := line[:star]
	computed := crc8(content)
	return bytes.TrimRight(content, " "), true, byte(claimed) == computed
}
