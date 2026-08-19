//go:build linux

package serial

// devicePatterns are the device nodes a USB serial adapter shows up as on
// Linux: ttyUSB* for FTDI/CP210x style bridges, ttyACM* for CDC devices such
// as the fdcanusb and the OpenRB-150.
var devicePatterns = []string{
	"/dev/ttyUSB*",
	"/dev/ttyACM*",
}
