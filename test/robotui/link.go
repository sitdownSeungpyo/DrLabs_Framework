package main

import (
	"bufio"
	"encoding/json"
	"math/rand"
	"net"
	"sync"
	"time"
)

// The client half of the servo protocol, as an upper layer would write it.
//
// This is the whole point of the exercise: the panel is not privileged.  It
// speaks the same newline-delimited JSON any other client would, over the same
// socket, and it gets the same refusals.  Nothing here can reach past the
// robot's safety envelope, because there is no path to.
//
// Every link drops, so the drop is modelled rather than treated as a failure:
// the connection is a state machine, reconnection backs off, and the panel is
// told which state it is in rather than being left to guess from silence.

// Connection states, reported to the browser as they are.
const (
	stateDisconnected = "disconnected"
	stateConnecting   = "connecting"
	stateReady        = "ready"
	stateLost         = "lost"
)

// jointState is one joint as the robot reports it.  Pointers are the reason
// this type exists rather than a map: a measurement the driver did not read is
// null, and a panel that renders a missing torque as 0.00 Nm is lying.
type jointState struct {
	Name      string   `json:"name"`
	Armed     bool     `json:"armed"`
	EStop     bool     `json:"estop"`
	Mode      string   `json:"mode"`
	Position  *float64 `json:"position_rad"`
	Velocity  *float64 `json:"velocity_rad_s"`
	Torque    *float64 `json:"torque_nm"`
	Target    *float64 `json:"target_rad"`
	Fault     int      `json:"fault"`
	FaultName string   `json:"fault_name"`
	RateHz    *float64 `json:"rate_hz"`
}

// eventLine is one entry from a joint's log.
//
// It is the half of the picture the state lines cannot carry.  A joint that
// trips latches on its own and then simply reads "e-stopped"; the reason it
// tripped only ever exists here, and a panel that drops it leaves the operator
// looking at a stopped machine with nothing to act on.
type eventLine struct {
	Joint   string `json:"joint"`
	Level   string `json:"level"`
	Message string `json:"message"`
	Detail  string `json:"detail"`
	Seq     uint64 `json:"seq"`
}

// eventHistory is how many log lines the panel keeps.  Enough to hold the
// bring-up of a small machine and whatever went wrong after it.
const eventHistory = 60

type robotLink struct {
	addr string

	mu     sync.RWMutex
	state  string
	joints []jointState
	events []eventLine

	sendMu sync.Mutex
	conn   net.Conn
	writer *bufio.Writer

	replies chan map[string]any
}

func newRobotLink(addr string) *robotLink {
	return &robotLink{addr: addr, state: stateDisconnected, replies: make(chan map[string]any, 8)}
}

// run keeps the connection up for the life of the process.
func (l *robotLink) run() {
	delay := 250 * time.Millisecond
	for {
		l.setState(stateConnecting)
		conn, err := net.DialTimeout("tcp", l.addr, 2*time.Second)
		if err != nil {
			l.setState(stateLost)
			time.Sleep(jitter(delay))
			if delay < 5*time.Second {
				delay *= 2
			}
			continue
		}
		delay = 250 * time.Millisecond

		l.sendMu.Lock()
		l.conn, l.writer = conn, bufio.NewWriter(conn)
		l.sendMu.Unlock()
		l.setState(stateReady)

		l.read(conn)

		l.sendMu.Lock()
		l.conn, l.writer = nil, nil
		l.sendMu.Unlock()
		_ = conn.Close()

		l.mu.Lock()
		l.joints = nil
		l.mu.Unlock()

		l.setState(stateLost)
		time.Sleep(jitter(delay))
	}
}

func (l *robotLink) read(conn net.Conn) {
	scanner := bufio.NewScanner(conn)
	scanner.Buffer(make([]byte, 0, 8192), 1<<20)
	for scanner.Scan() {
		line := scanner.Bytes()
		if len(line) == 0 {
			continue
		}
		var envelope struct {
			Type   string       `json:"type"`
			Joints []jointState `json:"joints"`
		}
		if err := json.Unmarshal(line, &envelope); err != nil {
			continue
		}
		switch envelope.Type {
		case "state":
			l.mu.Lock()
			l.joints = envelope.Joints
			l.mu.Unlock()
		case "event":
			var event eventLine
			if json.Unmarshal(line, &event) == nil {
				l.mu.Lock()
				l.events = append(l.events, event)
				if len(l.events) > eventHistory {
					l.events = l.events[len(l.events)-eventHistory:]
				}
				l.mu.Unlock()
			}
		case "reply":
			var reply map[string]any
			if json.Unmarshal(line, &reply) == nil {
				select {
				case l.replies <- reply:
				default: // nobody is waiting; a refusal still reaches the log
				}
			}
		}
	}
}

// send writes one command and waits for its reply.  Replies come back in the
// order the commands went out, which the protocol guarantees, so serialising
// the whole exchange is all the correlation that is needed.
func (l *robotLink) send(command map[string]any) (map[string]any, error) {
	payload, err := json.Marshal(command)
	if err != nil {
		return nil, err
	}

	l.sendMu.Lock()
	defer l.sendMu.Unlock()
	if l.writer == nil {
		return nil, errNotConnected
	}

	// Drain anything left over, so a reply cannot be matched to a command
	// that timed out earlier.
	for {
		select {
		case <-l.replies:
			continue
		default:
		}
		break
	}

	if _, err := l.writer.Write(append(payload, '\n')); err != nil {
		return nil, err
	}
	if err := l.writer.Flush(); err != nil {
		return nil, err
	}

	select {
	case reply := <-l.replies:
		return reply, nil
	case <-time.After(2 * time.Second):
		return nil, errTimeout
	}
}

func (l *robotLink) snapshot() (string, []jointState, []eventLine) {
	l.mu.RLock()
	defer l.mu.RUnlock()
	joints := make([]jointState, len(l.joints))
	copy(joints, l.joints)
	events := make([]eventLine, len(l.events))
	copy(events, l.events)
	return l.state, joints, events
}

func (l *robotLink) setState(state string) {
	l.mu.Lock()
	l.state = state
	l.mu.Unlock()
}

func jitter(d time.Duration) time.Duration {
	return d/2 + time.Duration(rand.Int63n(int64(d/2)+1))
}

type linkError string

func (e linkError) Error() string { return string(e) }

const (
	errNotConnected = linkError("not connected to the robot")
	errTimeout      = linkError("the robot did not answer")
)
