#!/usr/bin/env python3
"""A test client for `drlabs bringup` — the upper layer, addressing joints by name.

The robot server speaks newline-delimited JSON over TCP, in joint units.

    python robot.py state                 every joint, once
    python robot.py watch                 stream it until Ctrl+C
    python robot.py joints                the joint names
    python robot.py pos r_sh_p 0.2        move one joint, in radians
    python robot.py move r_sh_p 0.3       profiled move, with a velocity limit
    python robot.py pose r_sh_p=0.1 r_elb_p=-0.4
    python robot.py sweep r_sh_p 0.15 3   three there-and-back moves
    python robot.py hold r_sh_p / stop r_sh_p
    python robot.py estop / reset

Motion commands wait for the joint to settle and report the error against what
was asked, so a bench test is one line and a readable answer.
"""

import argparse
import json
import socket
import sys
import time


class RobotClient:
    def __init__(self, host, port, timeout=5.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.stream = self.sock.makefile('rwb')
        self.joints = {}

    def close(self):
        try:
            self.stream.close()
            self.sock.close()
        except OSError:
            pass

    def send(self, **command):
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
            raise ConnectionError('the robot closed the connection')
        message = json.loads(line)
        if message.get('type') == 'state':
            self.joints = {j['name']: j for j in message['joints']}
        return message

    def wait_state(self):
        while True:
            if self._read().get('type') == 'state':
                return self.joints

    def settle(self, names, seconds=5.0, still=0.02):
        """Wait until every named joint has stopped, or the time runs out."""
        deadline = time.time() + seconds
        quiet_since = None
        while time.time() < deadline:
            joints = self.wait_state()
            if any(joints[n].get('estop') for n in names if n in joints):
                return joints
            moving = False
            for name in names:
                velocity = joints.get(name, {}).get('velocity_rad_s')
                if velocity is None or abs(velocity) >= still:
                    moving = True
            if moving:
                quiet_since = None
            else:
                quiet_since = quiet_since or time.time()
                if time.time() - quiet_since > 0.3:
                    return joints
        return self.joints


def show(joint):
    def number(key, digits):
        value = joint.get(key)
        return '—' if value is None else ('%+.*f' % (digits, value))
    line = ('%-10s armed=%-5s %-9s %s rad  %s rad/s  %s Nm  %.0f Hz'
            % (joint['name'], joint['armed'], joint['mode'],
               number('position_rad', 4), number('velocity_rad_s', 3),
               number('torque_nm', 3), joint.get('rate_hz', 0)))
    if joint.get('estop'):
        line += '  ESTOP: ' + (joint.get('estop_reason') or '')
    return line


def report(joints, targets):
    for name, target in targets.items():
        joint = joints.get(name)
        if joint is None or joint.get('position_rad') is None:
            print('   %-10s no position reported' % name)
            continue
        error = joint['position_rad'] - target
        if joint.get('estop'):
            print('   %-10s stopped: %s' % (name, joint.get('estop_reason')))
        elif abs(error) < 0.01:
            print('   %-10s %+.4f rad   arrived, error %+.4f' % (name, joint['position_rad'], error))
        else:
            print('   %-10s %+.4f rad   did NOT arrive: off by %+.4f — torque limit '
                  'too low, or the joint is obstructed' % (name, joint['position_rad'], error))


def ready(client, names):
    """Refuse to command a joint that is not live, and say why."""
    joints = client.wait_state()
    blocked = False
    for name in names:
        joint = joints.get(name)
        if joint is None:
            print('no joint named %r — have: %s' % (name, ', '.join(joints)))
            return False
        if joint.get('estop'):
            print('%s: emergency stop latched — %s' % (name, joint.get('estop_reason')))
            print('clear it with:  python robot.py reset')
            blocked = True
        elif not joint['armed']:
            print('%s is not armed:' % name)
            for reason in joint.get('blocked') or []:
                print('  -', reason)
            blocked = True
    return not blocked


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('command')
    parser.add_argument('args', nargs='*')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=7777)
    parser.add_argument('--vel', type=float, default=0.5, help='velocity limit, rad/s')
    parser.add_argument('--accel', type=float, default=2.0, help='acceleration, rad/s^2')
    parser.add_argument('--settle', type=float, default=5.0)
    options = parser.parse_args()

    try:
        client = RobotClient(options.host, options.port)
    except OSError as err:
        print('Cannot reach the robot at %s:%d — is `drlabs bringup` running?'
              % (options.host, options.port))
        print('  %s' % err)
        return 1

    try:
        return run(client, options)
    except (RuntimeError, ConnectionError) as err:
        print('refused: %s' % err)
        return 1
    finally:
        client.close()


def run(client, options):
    command, values = options.command, options.args

    if command == 'joints':
        print('\n'.join(client.send(cmd='joints')['value']))
        return 0

    if command == 'state':
        for joint in client.wait_state().values():
            print(show(joint))
        return 0

    if command == 'watch':
        print('streaming state, Ctrl+C to stop')
        try:
            while True:
                joints = client.wait_state()
                print('\r' + ' | '.join(
                    '%s %+.4f' % (n, j['position_rad'] or 0.0) for n, j in joints.items()),
                    end='', flush=True)
        except KeyboardInterrupt:
            print()
        return 0

    if command == 'estop':
        client.send(cmd='estop', reason='raised from robot.py')
        print('emergency stop latched on every joint')
        return 0

    if command == 'reset':
        client.send(cmd='reset')
        time.sleep(1.5)
        for joint in client.wait_state().values():
            print(show(joint))
        return 0

    if command in ('hold', 'stop'):
        if not values:
            print('usage: robot.py %s <joint>' % command)
            return 2
        client.send(cmd=command, joint=values[0])
        time.sleep(0.3)
        print(show(client.wait_state()[values[0]]))
        return 0

    if command in ('pos', 'move'):
        if len(values) < 2:
            print('usage: robot.py %s <joint> <radians>' % command)
            return 2
        name, target = values[0], float(values[1])
        if not ready(client, [name]):
            return 1
        if command == 'pos':
            print('-> %s to %+.4f rad' % (name, target))
            client.send(cmd='position', joint=name, rad=target)
        else:
            print('-> %s profiled to %+.4f rad at %.2f rad/s' % (name, target, options.vel))
            client.send(cmd='move', joint=name, rad=target,
                        max_velocity=options.vel, accel=options.accel)
        report(client.settle([name], options.settle), {name: target})
        return 0

    if command == 'pose':
        if not values:
            print('usage: robot.py pose <joint>=<rad> [<joint>=<rad> ...]')
            return 2
        targets = {}
        for pair in values:
            name, _, radians = pair.partition('=')
            targets[name] = float(radians)
        if not ready(client, list(targets)):
            return 1
        print('-> pose: ' + ', '.join('%s %+.3f' % (n, v) for n, v in targets.items()))
        client.send(cmd='pose', joints=targets)
        report(client.settle(list(targets), options.settle), targets)
        return 0

    if command == 'sweep':
        if len(values) < 2:
            print('usage: robot.py sweep <joint> <amplitude-rad> [cycles]')
            return 2
        name, amplitude = values[0], float(values[1])
        cycles = int(values[2]) if len(values) > 2 else 3
        if not ready(client, [name]):
            return 1

        origin = client.joints[name].get('position_rad') or 0.0
        print('-> %d sweeps of %+.3f rad around %+.4f' % (cycles, amplitude, origin))
        for _ in range(cycles):
            for target in (origin + amplitude, origin - amplitude):
                client.send(cmd='move', joint=name, rad=target,
                            max_velocity=options.vel, accel=options.accel)
                joints = client.settle([name], options.settle)
                if joints[name].get('estop'):
                    print('   stopped: %s' % joints[name].get('estop_reason'))
                    return 1
                print('   %+.4f rad  (asked %+.4f)' % (joints[name]['position_rad'], target))
        client.send(cmd='move', joint=name, rad=origin,
                    max_velocity=options.vel, accel=options.accel)
        client.settle([name], options.settle)
        print(show(client.joints[name]))
        return 0

    print('unknown command %r — see --help' % command)
    return 2


if __name__ == '__main__':
    sys.exit(main())
