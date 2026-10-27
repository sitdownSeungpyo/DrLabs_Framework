"""The same bytes the Rust tests pin (crates/drlabs-api/src/frame.rs)."""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from drlabs_client.codec import (  # noqa: E402
    CMD, DESCRIBE, DESCRIBE_REPLY, DRIVE, STATE, FrameError, LowCmd, LowState,
    MotorCmd, MotorState, decode, encode_cmd, encode_describe,
    encode_describe_reply, encode_state,
)

CMD_VECTOR = (
    "444c525401000100"
    "070000000200000008070605040302010200000000000000"
    "010000000000003f000080bf000020410000003f0000803e"
    "000000000000000000000000000000000000000000000000"
    "1b4b94a0"
)
STATE_VECTOR = (
    "444c525401000200"
    "090000000700000040e2010000000000887766554433221180841e00030000000400010002000000"
    "0000003f000000000000c07fcdcccc3d000040410000f84120000200"
    "0000c07f0000c07f0000c07f0000c07f0000c07f0000c07f00000400"
    "fd45c257"
)


def sample_cmd() -> LowCmd:
    return LowCmd(seq=7, flags=2, t_ns=0x0102030405060708, cmd=[
        MotorCmd(mode=DRIVE, q=0.5, dq=-1.0, kp=10.0, kd=0.5, tau=0.25),
        MotorCmd(),
    ])


def sample_state() -> LowState:
    return LowState(seq=9, cmd_seq=7, tick=123456, t_ns=0x1122334455667788,
                    period_ns=2_000_000, deadline_miss=3, estop_code=4, estop=1, state=[
                        MotorState(q=0.5, dq=0.0, tau=math.nan, current=0.1, voltage=12.0,
                                   temp=31.0, fault=0x20, status=2, blocked=0),
                        MotorState(status=4),
                    ])


def test_command_vector_matches_rust():
    assert encode_cmd(sample_cmd()).hex() == CMD_VECTOR


def test_state_vector_matches_rust():
    assert encode_state(sample_state()).hex() == STATE_VECTOR


def test_round_trip():
    kind, cmd = decode(encode_cmd(sample_cmd()))
    assert kind == CMD and cmd == sample_cmd()
    kind, state = decode(encode_state(sample_state()))
    assert kind == STATE
    assert state.tick == 123456 and math.isnan(state.state[0].tau)
    assert state.state[1].status == 4 and state.estop_name == "over_current"
    assert decode(encode_describe()) == (DESCRIBE, None)
    assert decode(encode_describe_reply('{"robot":"x"}')) == (DESCRIBE_REPLY, '{"robot":"x"}')


def test_damage_is_refused():
    data = bytearray(encode_cmd(sample_cmd()))
    data[-1] ^= 1
    try:
        decode(bytes(data))
    except FrameError as error:
        assert "CRC" in str(error)
    else:
        raise AssertionError("a bad CRC was accepted")


if __name__ == "__main__":
    for name, test in list(globals().items()):
        if name.startswith("test_"):
            test()
            print("ok", name)
