// Command webui serves a browser control panel for one motor.
//
// It is the framework's runtime with an HTTP surface bolted on: the session
// owns the control loop, and this process only translates requests into
// session calls and pushes snapshots back out.  Nothing here touches a bus, so
// a slow browser, a reloaded page or a closed tab cannot inject jitter into
// motor control.
//
//	go run ./test/webui                                    simulator
//	go run ./test/webui -driver moteus -port COM5 -id 1     hardware
//
// Then open http://localhost:8080.
//
// The connection is made from the browser, not from these flags — the flags
// only pre-fill the form, so the page can be left open across reconnects.
package main

import (
	"context"
	"flag"
	"fmt"
	"log"
	"net"
	"net/http"
	"os"
	"os/signal"
	"syscall"
	"time"

	"drlabs/runtime/session"

	_ "drlabs/drivers/dynamixel"
	_ "drlabs/drivers/moteus"
	_ "drlabs/drivers/robstride"
)

func main() {
	addr := flag.String("addr", "127.0.0.1:8080", "address to listen on")
	rate := flag.Int("rate", 200, "initial control rate in Hz")

	// Defaults for the connect form.
	driver := flag.String("driver", "moteus-sim", "driver to pre-select")
	port := flag.String("port", "", "serial port to pre-fill")
	id := flag.Int("id", 1, "bus address to pre-fill")
	model := flag.String("model", "", "RobStride model to pre-fill")
	flag.Parse()

	srv := &server{
		session: session.New(session.Options{RateHz: *rate}),
		defaults: connectRequest{
			Driver: *driver,
			Port:   *port,
			ID:     *id,
			Model:  *model,
			RateHz: *rate,
		},
	}

	mux := http.NewServeMux()
	srv.routes(mux)

	httpServer := &http.Server{
		Addr:    *addr,
		Handler: mux,
		// The telemetry stream is long-lived, so there is no write deadline;
		// the read side is bounded because request bodies are small.
		ReadHeaderTimeout: 5 * time.Second,
	}

	// Bind before announcing, so the printed URL is never a lie.
	listener, err := net.Listen("tcp", *addr)
	if err != nil {
		log.Fatalf("webui: cannot listen on %s: %v", *addr, err)
	}

	fmt.Printf("DrLabs web UI on http://%s\n", listener.Addr())
	fmt.Printf("  driver %s · id %d%s\n", *driver, *id, portSuffix(*port))
	fmt.Println("  Ctrl+C to stop")

	signals := make(chan os.Signal, 1)
	signal.Notify(signals, os.Interrupt, syscall.SIGTERM)

	go func() {
		if err := httpServer.Serve(listener); err != nil && err != http.ErrServerClosed {
			log.Printf("webui: %v", err)
		}
	}()

	<-signals
	fmt.Println("\nStopping…")

	// Stop the axis before the process goes away.  A web UI that leaves a
	// motor driving when it exits is worse than one that never started.
	srv.session.Disarm("web UI shutting down")
	_ = srv.session.Disconnect()

	shutdownCtx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	_ = httpServer.Shutdown(shutdownCtx)
}

func portSuffix(port string) string {
	if port == "" {
		return ""
	}
	return " · " + port
}
