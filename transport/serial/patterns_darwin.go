//go:build darwin

package serial

// devicePatterns lists only the callout (/dev/cu.*) side of each device.
// The /dev/tty.* twin blocks on open until carrier detect is asserted, which
// a USB CDC adapter never does — opening it looks like a hung connection.
var devicePatterns = []string{
	"/dev/cu.usbmodem*",
	"/dev/cu.usbserial*",
	"/dev/cu.SLAB_USBtoUART*",
	"/dev/cu.wchusbserial*",
}
