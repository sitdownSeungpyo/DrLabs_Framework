"""The bytes the Rust encoder (crates/drlabs-api/src/cdr.rs) pins, produced
here by the Eclipse ``cyclonedds`` package from the same ROS 2-named types.

The two sides never exchange these buffers directly — Cyclone does — so a
drift between them would show up as a ROS 2 node silently ignoring the
runtime. The FNV-1a digests below are asserted on both sides.

    python -m pytest tools/drlabs_client/tests/test_cdr.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from drlabs_client.types import (  # noqa: E402
    DRIVE, Describe, LowCmd, LowState, MotorCmd, MotorState,
)

CDR_LE = bytes.fromhex("00010000")


def fnv1a64(data: bytes) -> int:
    h = 0xCBF29CE484222325
    for b in data:
        h ^= b
        h = (h * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return h


def sample_cmd() -> LowCmd:
    cmd = LowCmd(seq=7, flags=2, t_ns=0x0102030405060708, n=2)
    cmd.cmd[0] = MotorCmd(mode=DRIVE, q=0.5, dq=-1.0, kp=10.0, kd=0.5, tau=0.25)
    return cmd


def sample_state() -> LowState:
    state = LowState(seq=9, cmd_seq=7, tick=123456, t_ns=0x1122334455667788,
                     period_ns=2_000_000, deadline_miss=3, estop_code=4, estop=1, n=2)
    # Entries past n: the Rust record's EMPTY (NaN everywhere, status 0).
    state.state = [MotorState(status=0) for _ in range(64)]
    state.state[0] = MotorState(q=0.5, dq=0.0, tau=math.nan, current=0.1, voltage=12.0,
                                temp=31.0, fault=0x20, status=2, blocked=0)
    state.state[1] = MotorState(status=4)
    return state


def test_a_command_opens_with_the_ros2_header_and_body_relative_alignment():
    data = sample_cmd().serialize()
    assert data[:4] == CDR_LE
    head = ("07000000" "02000000" "0807060504030201" "0200" "01" "00" "0000003f")
    assert data[4:4 + len(head) // 2].hex() == head
    assert len(data) == 4 + 18 + 22 + 63 * 24


def test_a_state_opens_with_its_header_fields_packed_as_ros2_packs_them():
    data = sample_state().serialize()
    head = ("09000000" "07000000" "40e2010000000000" "8877665544332211" "80841e00"
            "03000000" "0400" "01" "00" "0200" "0000" "0000003f")
    assert data[:4] == CDR_LE
    assert data[4:4 + len(head) // 2].hex() == head, data[:48].hex()


def test_a_description_is_a_counted_nul_terminated_string():
    data = Describe(json='{"a":1}').serialize()
    assert data.hex() == "00010000" "08000000" "7b2261223a317d" "00"


def test_the_whole_buffers_match_the_rust_encoder():
    """The same two digests are asserted in crates/drlabs-api/src/cdr.rs."""
    assert fnv1a64(sample_cmd().serialize()) == RUST_CMD_FNV
    assert fnv1a64(sample_state().serialize()) == RUST_STATE_FNV


# Filled in from the Rust test `whole_buffer_digests_are_pinned_for_python`.
RUST_CMD_FNV = 0x452AEDBF1163B77C
RUST_STATE_FNV = 0x6EE06EDEB2FA4FD9


if __name__ == "__main__":
    print("cmd  fnv1a64 = 0x%016x" % fnv1a64(sample_cmd().serialize()))
    print("state fnv1a64 = 0x%016x" % fnv1a64(sample_state().serialize()))
