#!/usr/bin/env python3
"""A test client for `drlabs serve` — the upper layer, in one small script.

The servo speaks newline-delimited JSON over TCP.  This is thin enough to read
in one sitting and to copy into whatever ends up giving the real commands.

    python drive.py state              print one state line and exit
    python drive.py watch              stream state until Ctrl+C
    python drive.py pos 0.05           move to 0.05 revolutions
    python drive.py vel 0.2            spin at 0.2 rev/s
    python drive.py traj 0.1           profiled move, with a velocity limit
    python drive.py hold               hold position, no target
    python drive.py stop               release the bridge
    python drive.py estop / reset      latch / clear the emergency stop
    python drive.py sweep 0.05 3       three there-and-back moves of ±0.05 rev

Every motion command waits for the axis to settle and prints what happened, so
a first bench test is one line and a readable answer rather than a guess.
"""

import argparse
import json
import socket
import sys
import time


class Servo:
    def __init__(self, host, port, timeout=5.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.stream = self.sock.makefile('rwb')
        self.state = None

    def close(self):
        try:
            self.stream.close()
            self.sock.close()
        except OSError:
            pass

    def send(self, **command):
        """Send one command and return its reply.

        State lines arrive on the same socket, so anything that is not a reply
        is folded into the latest state rather than discarded — otherwise the
        next read would hand back a stale one.
        """
        self.stream.write((json.dumps(command) + '\n').encode())
        self.stream.flush()
        while True:
            message = self._read()
            if message.get('type') == 'reply':
                if not message.get('ok'):
                    raise RuntimeError(message.get('error', 'refused'))
                return message

    def _read(self):
        line = self.stream.readline()
        if not line:
            raise ConnectionError('the servo closed the connection')
        message = json.loads(line)
        if message.get('type') == 'state':
            self.state = message
        return message

    def wait_state(self):
        """Block until a fresh state line arrives."""
        while True:
            if self._read().get('type') == 'state':
                return self.state

    def settle(self, seconds=4.0, tolerance=0.002, still=0.01):
        """Wait for the axis to stop moving, or until the time runs out.

        Returns the last state either way; the caller reports what it sees
        rather than this raising, because "it did not get there" is a result a
        bench test wants printed, not an exception.
        """
        deadline = time.time() + seconds
        settled_since = None
        while time.time() < deadline:
            state = self.wait_state()
            if state.get('estop'):
                return state
            velocity = state.get('velocity')
            if velocity is not None and abs(velocity) < still:
                settled_since = settled_since or time.time()
                if time.time() - settled_since > 0.3:
                    return state
            else:
                settled_since = None
        return self.state


def describe(state):
    if state is None:
        return 'no state received'
    def number(key, digits):
        value = state.get(key)
        return '—' if value is None else ('%+.*f' % (digits, value))
    return ('armed=%-5s mode=%-9s pos=%s rev  vel=%s rev/s  %s A  %s V  %s C  %.0f Hz'
            % (state['armed'], state['mode'], number('position', 4),
               number('velocity', 3), number('current', 2),
               number('voltage', 1), number('temperature', 1),
               state.get('rate_hz', 0)))


def check(servo):
    """Refuse to command motion into a machine that is not ready for it."""
    state = servo.wait_state()
    if state.get('estop'):
        print('Emergency stop is latched: %s' % state.get('estop_reason'))
        print('Clear it with:  python drive.py reset')
        return None
    if not state['armed']:
        print('The axis is not armed.')
        for reason in state.get('blocked') or []:
            print('  -', reason)
        return None
    return state


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('command')
    parser.add_argument('args', nargs='*', type=float)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=7777)
    parser.add_argument('--vel', type=float, default=0.2,
                        help='velocity limit for traj and sweep, rev/s')
    parser.add_argument('--accel', type=float, default=1.0,
                        help='acceleration for traj, rev/s^2')
    parser.add_argument('--settle', type=float, default=4.0,
                        help='seconds to wait for the axis to stop')
    options = parser.parse_args()

    try:
        servo = Servo(options.host, options.port)
    except OSError as err:
        print('Cannot reach the servo at %s:%d — is `drlabs serve` running?'
              % (options.host, options.port))
        print('  %s' % err)
        return 1

    try:
        return run(servo, options)
    except (RuntimeError, ConnectionError) as err:
        print('refused: %s' % err)
        return 1
    finally:
        servo.close()


def run(servo, options):
    command = options.command
    values = options.args

    if command == 'state':
        print(describe(servo.wait_state()))
        return 0

    if command == 'watch':
        print('streaming state, Ctrl+C to stop')
        try:
            while True:
                print('\r' + describe(servo.wait_state()), end='', flush=True)
        except KeyboardInterrupt:
            print()
        return 0

    if command in ('stop', 'hold'):
        servo.send(cmd=command)
        time.sleep(0.3)
        print(describe(servo.wait_state()))
        return 0

    if command == 'estop':
        servo.send(cmd='estop')
        print('emergency stop latched')
        return 0

    if command == 'reset':
        servo.send(cmd='reset')
        time.sleep(1.0)
        print(describe(servo.wait_state()))
        return 0

    if command == 'pos':
        if not values:
            print('usage: drive.py pos <revolutions>')
            return 2
        if check(servo) is None:
            return 1
        target = values[0]
        print('-> position %+.4f rev' % target)
        servo.send(cmd='position', value=target)
        state = servo.settle(options.settle)
        print(describe(state))
        report_error(state, target)
        return 0

    if command == 'vel':
        if not values:
            print('usage: drive.py vel <rev/s>')
            return 2
        if check(servo) is None:
            return 1
        target = values[0]
        print('-> velocity %+.3f rev/s for 2 s' % target)
        # Held open deliberately: the servo reverts to a position hold about a
        # second after the last command, which is the guard being demonstrated.
        deadline = time.time() + 2.0
        while time.time() < deadline:
            servo.send(cmd='velocity', value=target)
            time.sleep(0.2)
        servo.send(cmd='hold')
        print(describe(servo.settle(options.settle)))
        return 0

    if command == 'traj':
        if not values:
            print('usage: drive.py traj <revolutions> [--vel v] [--accel a]')
            return 2
        if check(servo) is None:
            return 1
        target = values[0]
        print('-> trajectory to %+.4f rev at %.2f rev/s, %.2f rev/s^2'
              % (target, options.vel, options.accel))
        servo.send(cmd='trajectory', target=target,
                   max_velocity=options.vel, accel=options.accel)
        peak = 0.0
        deadline = time.time() + options.settle + 10
        while time.time() < deadline:
            state = servo.wait_state()
            peak = max(peak, abs(state.get('velocity') or 0.0))
            if state.get('estop'):
                break
            if not state['trajectory_running'] and state['trajectory_progress'] >= 1.0:
                break
        state = servo.settle(1.5)
        print(describe(state))
        print('   peak speed %.3f rev/s against a %.2f limit' % (peak, options.vel))
        report_error(state, target)
        return 0

    if command == 'sweep':
        if len(values) < 1:
            print('usage: drive.py sweep <amplitude-rev> [cycles]')
            return 2
        if check(servo) is None:
            return 1
        amplitude = values[0]
        cycles = int(values[1]) if len(values) > 1 else 3
        origin = servo.state.get('position') or 0.0
        print('-> %d sweeps of %+.4f rev around %+.4f' % (cycles, amplitude, origin))
        for index in range(cycles):
            for target in (origin + amplitude, origin - amplitude):
                servo.send(cmd='trajectory', target=target,
                           max_velocity=options.vel, accel=options.accel)
                deadline = time.time() + options.settle + 10
                while time.time() < deadline:
                    state = servo.wait_state()
                    if state.get('estop'):
                        print(describe(state))
                        print('stopped: %s' % state.get('estop_reason'))
                        return 1
                    if not state['trajectory_running'] and state['trajectory_progress'] >= 1.0:
                        break
                state = servo.settle(1.5)
                print('   %+.4f rev  (asked %+.4f)' % (state.get('position') or 0.0, target))
        servo.send(cmd='trajectory', target=origin,
                   max_velocity=options.vel, accel=options.accel)
        servo.settle(options.settle)
        print(describe(servo.state))
        return 0

    print('unknown command %r — see --help' % command)
    return 2


def report_error(state, target):
    position = state.get('position')
    if position is None:
        return
    error = position - target
    if state.get('estop'):
        print('   stopped before arriving: %s' % state.get('estop_reason'))
    elif abs(error) < 0.005:
        print('   arrived, error %+.4f rev' % error)
    else:
        print('   did NOT arrive: off by %+.4f rev — torque limit too low, or the axis is obstructed'
              % error)


if __name__ == '__main__':
    sys.exit(main())
