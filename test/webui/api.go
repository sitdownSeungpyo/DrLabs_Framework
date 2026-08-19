package main

import (
	"context"
	"embed"
	"encoding/json"
	"fmt"
	"io/fs"
	"net/http"
	"time"

	"drlabs/motor"
	"drlabs/runtime/session"
	"drlabs/transport/serial"
)

//go:embed static
var staticFiles embed.FS

// server owns the one session this page controls.
type server struct {
	session  *session.Session
	defaults connectRequest
}

func (s *server) routes(mux *http.ServeMux) {
	static, err := fs.Sub(staticFiles, "static")
	if err != nil {
		panic(err)
	}
	mux.Handle("/", http.FileServer(http.FS(static)))

	mux.HandleFunc("/api/defaults", s.handleDefaults)
	mux.HandleFunc("/api/drivers", s.handleDrivers)
	mux.HandleFunc("/api/ports", s.handlePorts)
	mux.HandleFunc("/api/stream", s.handleStream)

	mux.HandleFunc("/api/connect", post(s.handleConnect))
	mux.HandleFunc("/api/disconnect", post(s.handleDisconnect))
	mux.HandleFunc("/api/arm", post(s.handleArm))
	mux.HandleFunc("/api/disarm", post(s.handleDisarm))
	mux.HandleFunc("/api/estop", post(s.handleEStop))
	mux.HandleFunc("/api/estop/reset", post(s.handleEStopReset))
	mux.HandleFunc("/api/mode", post(s.handleMode))
	mux.HandleFunc("/api/setpoint", post(s.handleSetpoint))
	mux.HandleFunc("/api/params", post(s.handleParams))
	mux.HandleFunc("/api/safety", post(s.handleSafety))
	mux.HandleFunc("/api/preset", post(s.handlePreset))
	mux.HandleFunc("/api/rate", post(s.handleRate))
	mux.HandleFunc("/api/zero", post(s.handleZero))
	mux.HandleFunc("/api/shuttle/start", post(s.handleShuttleStart))
	mux.HandleFunc("/api/shuttle/stop", post(s.handleShuttleStop))
	mux.HandleFunc("/api/trajectory/start", post(s.handleTrajectoryStart))
	mux.HandleFunc("/api/trajectory/stop", post(s.handleTrajectoryStop))
}

// post wraps a handler so that only POST reaches it.  Every control endpoint
// changes machine state, and none of them belongs behind a URL a browser might
// prefetch.
func post(h func(http.ResponseWriter, *http.Request)) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodPost {
			http.Error(w, "POST required", http.StatusMethodNotAllowed)
			return
		}
		h(w, r)
	}
}

func writeJSON(w http.ResponseWriter, value any) {
	w.Header().Set("Content-Type", "application/json")
	_ = json.NewEncoder(w).Encode(value)
}

// fail reports a refusal to the page.
//
// The HTTP status stays 200 because these are answers the operator needs to
// read — "arm the axis first" is a result, not a transport failure, and a 4xx
// would have the browser treat it as a broken request instead of showing it.
func fail(w http.ResponseWriter, err error) {
	writeJSON(w, map[string]any{"ok": false, "error": err.Error()})
}

func ok(w http.ResponseWriter) { writeJSON(w, map[string]any{"ok": true}) }

func decode(r *http.Request, into any) error {
	defer r.Body.Close()
	return json.NewDecoder(r.Body).Decode(into)
}

// --- read-only endpoints ---------------------------------------------------

type connectRequest struct {
	Driver            string `json:"driver"`
	Port              string `json:"port"`
	Baud              int    `json:"baud"`
	ID                int    `json:"id"`
	Model             string `json:"model"`
	RateHz            int    `json:"rate_hz"`
	AllowUnknownBoard bool   `json:"allow_unknown_board"`
}

func (s *server) handleDefaults(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, s.defaults)
}

func (s *server) handleDrivers(w http.ResponseWriter, r *http.Request) {
	type entry struct {
		Name        string `json:"name"`
		Description string `json:"description"`
	}
	out := []entry{}
	for _, name := range motor.Drivers() {
		info, _ := motor.DriverInfo(name)
		out = append(out, entry{info.Name, info.Description})
	}
	writeJSON(w, out)
}

func (s *server) handlePorts(w http.ResponseWriter, r *http.Request) {
	ports, err := serial.List()
	if err != nil {
		fail(w, err)
		return
	}
	if ports == nil {
		ports = []serial.PortInfo{}
	}
	writeJSON(w, ports)
}

// handleStream pushes status snapshots as server-sent events.
//
// SSE rather than a websocket because it needs no dependency and no handshake,
// and the traffic is one-way: the page reads state and posts commands
// separately.  The interval is deliberately far slower than the control rate —
// a browser has no use for 200 updates a second, and the loop should not be
// paying to serialise them.
func (s *server) handleStream(w http.ResponseWriter, r *http.Request) {
	flusher, canFlush := w.(http.Flusher)
	if !canFlush {
		http.Error(w, "streaming unsupported", http.StatusInternalServerError)
		return
	}

	w.Header().Set("Content-Type", "text/event-stream")
	w.Header().Set("Cache-Control", "no-cache")
	w.Header().Set("Connection", "keep-alive")

	ticker := time.NewTicker(50 * time.Millisecond)
	defer ticker.Stop()

	var lastEvent uint64
	var lastSample uint64

	for {
		select {
		case <-r.Context().Done():
			return

		case <-ticker.C:
			// This is the deadman's heartbeat, and it deliberately comes from
			// the same place that is watching: a page that stops reading stops
			// the axis, which is the entire point of the guard.
			s.session.Heartbeat()

			status := s.session.Status()

			events := s.session.Events(lastEvent)
			for _, event := range events {
				lastEvent = event.Seq
			}

			samples := s.session.Samples(lastSample, 400)
			if len(samples) > 0 {
				lastSample = samples[len(samples)-1].Seq
			}

			payload, err := json.Marshal(snapshot{
				Status:  newStatusView(status),
				Events:  newEventViews(events),
				Samples: newSampleViews(samples),
			})
			if err != nil {
				return
			}
			if _, err := fmt.Fprintf(w, "data: %s\n\n", payload); err != nil {
				return
			}
			flusher.Flush()
		}
	}
}

// --- control endpoints -----------------------------------------------------

func (s *server) handleConnect(w http.ResponseWriter, r *http.Request) {
	var req connectRequest
	if err := decode(r, &req); err != nil {
		fail(w, err)
		return
	}

	cfg := motor.Config{
		Driver: req.Driver,
		Port:   req.Port,
		Baud:   req.Baud,
		ID:     req.ID,
	}
	if req.Model != "" {
		cfg.Options = map[string]string{"model": req.Model}
	}
	if req.RateHz > 0 {
		s.session.SetRate(req.RateHz)
	}

	// Connecting can take seconds — an adapter to open, a board to answer, a
	// console to drain — so it gets its own budget rather than the request's.
	ctx, cancel := context.WithTimeout(r.Context(), 30*time.Second)
	defer cancel()

	if err := s.session.Connect(ctx, cfg, req.AllowUnknownBoard); err != nil {
		fail(w, err)
		return
	}
	ok(w)
}

func (s *server) handleDisconnect(w http.ResponseWriter, r *http.Request) {
	if err := s.session.Disconnect(); err != nil {
		fail(w, err)
		return
	}
	ok(w)
}

func (s *server) handleArm(w http.ResponseWriter, r *http.Request) {
	var req struct {
		Acknowledged bool `json:"acknowledged"`
	}
	_ = decode(r, &req)
	if err := s.session.Arm(req.Acknowledged); err != nil {
		fail(w, err)
		return
	}
	ok(w)
}

func (s *server) handleDisarm(w http.ResponseWriter, r *http.Request) {
	s.session.Disarm("operator")
	ok(w)
}

func (s *server) handleEStop(w http.ResponseWriter, r *http.Request) {
	s.session.EStop("operator", "Emergency stop", "Raised from the web UI.")
	ok(w)
}

func (s *server) handleEStopReset(w http.ResponseWriter, r *http.Request) {
	if err := s.session.ResetEStop(); err != nil {
		fail(w, err)
		return
	}
	ok(w)
}

func (s *server) handleMode(w http.ResponseWriter, r *http.Request) {
	var req struct {
		Mode string `json:"mode"`
	}
	if err := decode(r, &req); err != nil {
		fail(w, err)
		return
	}
	mode, valid := motor.ParseControlMode(req.Mode)
	if !valid {
		fail(w, fmt.Errorf("unknown mode %q", req.Mode))
		return
	}
	if err := s.session.SetMode(mode); err != nil {
		fail(w, err)
		return
	}
	ok(w)
}

func (s *server) handleSetpoint(w http.ResponseWriter, r *http.Request) {
	// Pointers, so that a page updating only the position does not silently
	// reset the velocity setpoint to the zero value of a missing field.
	var req struct {
		Position    *float64 `json:"position"`
		Velocity    *float64 `json:"velocity"`
		WithinLower *float64 `json:"within_lower"`
		WithinUpper *float64 `json:"within_upper"`
	}
	if err := decode(r, &req); err != nil {
		fail(w, err)
		return
	}
	if req.Position != nil {
		s.session.SetPosition(*req.Position)
	}
	if req.Velocity != nil {
		s.session.SetVelocity(*req.Velocity)
	}
	if req.WithinLower != nil && req.WithinUpper != nil {
		s.session.SetWithin(*req.WithinLower, *req.WithinUpper)
	}
	ok(w)
}

func (s *server) handleParams(w http.ResponseWriter, r *http.Request) {
	// Start from the live values so a partial update leaves the rest alone.
	params := s.session.Params()
	if err := decode(r, &params); err != nil {
		fail(w, err)
		return
	}
	writeJSON(w, map[string]any{"ok": true, "params": s.session.SetParams(params)})
}

func (s *server) handleSafety(w http.ResponseWriter, r *http.Request) {
	config := s.session.Safety()
	if err := decode(r, &config); err != nil {
		fail(w, err)
		return
	}
	_, notes := s.session.SetSafety(config)
	writeJSON(w, map[string]any{"ok": true, "notes": notes})
}

func (s *server) handlePreset(w http.ResponseWriter, r *http.Request) {
	var req struct {
		Preset string `json:"preset"`
	}
	if err := decode(r, &req); err != nil {
		fail(w, err)
		return
	}
	if err := s.session.SetPreset(req.Preset); err != nil {
		fail(w, err)
		return
	}
	ok(w)
}

func (s *server) handleRate(w http.ResponseWriter, r *http.Request) {
	var req struct {
		RateHz int `json:"rate_hz"`
	}
	if err := decode(r, &req); err != nil {
		fail(w, err)
		return
	}
	writeJSON(w, map[string]any{"ok": true, "rate_hz": s.session.SetRate(req.RateHz)})
}

func (s *server) handleZero(w http.ResponseWriter, r *http.Request) {
	if err := s.session.ZeroOutput(0); err != nil {
		fail(w, err)
		return
	}
	ok(w)
}

func (s *server) handleShuttleStart(w http.ResponseWriter, r *http.Request) {
	var req struct {
		A      float64 `json:"a"`
		B      float64 `json:"b"`
		Dwell  float64 `json:"dwell"`
		Cycles int     `json:"cycles"`
	}
	if err := decode(r, &req); err != nil {
		fail(w, err)
		return
	}
	s.session.ConfigureShuttle(req.A, req.B, req.Dwell, req.Cycles)
	if err := s.session.StartShuttle(); err != nil {
		fail(w, err)
		return
	}
	ok(w)
}

func (s *server) handleShuttleStop(w http.ResponseWriter, r *http.Request) {
	s.session.StopShuttle()
	ok(w)
}

func (s *server) handleTrajectoryStart(w http.ResponseWriter, r *http.Request) {
	var req struct {
		Target      float64 `json:"target"`
		MaxVelocity float64 `json:"max_velocity"`
		Accel       float64 `json:"accel"`
	}
	if err := decode(r, &req); err != nil {
		fail(w, err)
		return
	}
	if err := s.session.StartTrajectory(req.Target, req.MaxVelocity, req.Accel); err != nil {
		fail(w, err)
		return
	}
	ok(w)
}

func (s *server) handleTrajectoryStop(w http.ResponseWriter, r *http.Request) {
	s.session.StopTrajectory()
	ok(w)
}
