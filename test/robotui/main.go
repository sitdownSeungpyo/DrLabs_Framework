// Command robotui is a browser control panel for a running `drlabs bringup`.
//
// It is an upper layer, and deliberately nothing more: it holds one slider per
// joint, sends poses down the same JSON socket any other client would use, and
// renders what comes back.  It owns no control loop, no limits and no state
// the robot does not report.
//
//	drlabs bringup test/shoulder_pair.json          # the robot
//	go run ./test/robotui -robot test/shoulder_pair.json
//
// Then open http://localhost:8080.
//
// The robot description is read for the joint names and travel limits, so the
// sliders cannot be dragged past the envelope — but that is a courtesy to the
// operator, not a safety measure.  The envelope that matters is enforced on
// the far side of the socket, and this panel gets refused there like anything
// else.
package main

import (
	"embed"
	"encoding/json"
	"flag"
	"fmt"
	"io/fs"
	"log"
	"net/http"
	"os"
	"os/signal"
	"syscall"
	"time"

	"drlabs/runtime/robot"
)

//go:embed static
var staticFiles embed.FS

type jointLimits struct {
	Name   string  `json:"name"`
	MinRad float64 `json:"min_rad"`
	MaxRad float64 `json:"max_rad"`
	MaxNm  float64 `json:"max_torque_nm"`
	Driver string  `json:"driver"`
	Bus    string  `json:"bus"`
	ID     int     `json:"id"`
}

type server struct {
	link   *robotLink
	config *robot.Config
	joints []jointLimits
}

func main() {
	robotPath := flag.String("robot", "", "robot description, for joint names and travel limits (required)")
	robotAddr := flag.String("addr", "127.0.0.1:7777", "address of the running bringup server")
	httpAddr := flag.String("http", "127.0.0.1:8080", "address to serve the panel on")
	flag.Parse()

	if *robotPath == "" {
		fmt.Fprintln(os.Stderr, "usage: robotui -robot <robot.json> [-addr host:port] [-http host:port]")
		os.Exit(2)
	}

	config, err := robot.Load(*robotPath)
	if err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}

	srv := &server{link: newRobotLink(*robotAddr), config: config}
	for _, joint := range config.Joints {
		srv.joints = append(srv.joints, jointLimits{
			Name:   joint.Name,
			MinRad: joint.Limits.MinRad,
			MaxRad: joint.Limits.MaxRad,
			MaxNm:  joint.Limits.MaxTorqueNm,
			Driver: joint.Driver,
			Bus:    joint.Bus,
			ID:     joint.ID,
		})
	}
	go srv.link.run()

	content, err := fs.Sub(staticFiles, "static")
	if err != nil {
		log.Fatal(err)
	}

	mux := http.NewServeMux()
	mux.Handle("/", http.FileServer(http.FS(content)))
	mux.HandleFunc("/api/robot", srv.handleRobot)
	mux.HandleFunc("/api/state", srv.handleState)
	mux.HandleFunc("/api/command", srv.handleCommand)

	httpServer := &http.Server{Addr: *httpAddr, Handler: mux}
	go func() {
		fmt.Printf("panel on http://%s  ->  robot at %s\n", *httpAddr, *robotAddr)
		fmt.Printf("  %s · %d joints · %d Hz\n", config.Robot, len(config.Joints), config.RateHz)
		if err := httpServer.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Fatal(err)
		}
	}()

	signals := make(chan os.Signal, 1)
	signal.Notify(signals, os.Interrupt, syscall.SIGTERM)
	<-signals
	fmt.Println("\nStopping…")
	_ = httpServer.Close()
}

// handleRobot describes the machine once, so the page can build its sliders.
func (s *server) handleRobot(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, map[string]any{
		"robot":  s.config.Robot,
		"rate":   s.config.RateHz,
		"joints": s.joints,
	})
}

// handleState streams the robot's own state.  Server-sent events, because the
// traffic is one way and a browser reconnects them by itself.
func (s *server) handleState(w http.ResponseWriter, r *http.Request) {
	flusher, ok := w.(http.Flusher)
	if !ok {
		http.Error(w, "streaming unsupported", http.StatusInternalServerError)
		return
	}
	w.Header().Set("Content-Type", "text/event-stream")
	w.Header().Set("Cache-Control", "no-cache")
	w.Header().Set("Connection", "keep-alive")

	// The robot reports at its own rate; the panel is flushed at a rate a
	// browser can actually render.  A 200 Hz machine does not need to repaint
	// a slider 200 times a second.
	ticker := time.NewTicker(50 * time.Millisecond)
	defer ticker.Stop()

	for {
		select {
		case <-r.Context().Done():
			return
		case <-ticker.C:
			state, joints, events := s.link.snapshot()
			payload, err := json.Marshal(map[string]any{
				"connection": state,
				"joints":     joints,
				"events":     events,
			})
			if err != nil {
				continue
			}
			fmt.Fprintf(w, "data: %s\n\n", payload)
			flusher.Flush()
		}
	}
}

// handleCommand forwards one command and returns the robot's own answer,
// refusals included.  A refusal is information the operator needs, not an
// error to be swallowed into a 500.
func (s *server) handleCommand(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "POST only", http.StatusMethodNotAllowed)
		return
	}
	var command map[string]any
	if err := json.NewDecoder(r.Body).Decode(&command); err != nil {
		writeJSON(w, map[string]any{"ok": false, "error": "malformed command"})
		return
	}
	reply, err := s.link.send(command)
	if err != nil {
		writeJSON(w, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	writeJSON(w, reply)
}

func writeJSON(w http.ResponseWriter, value any) {
	w.Header().Set("Content-Type", "application/json")
	_ = json.NewEncoder(w).Encode(value)
}
