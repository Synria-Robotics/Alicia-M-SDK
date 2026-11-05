"""0x06 ACK/telemetry dispatch, with no serial port and no threads."""
import unittest
from unittest.mock import patch

from alicia_m_sdk.hardware.codec import MessageCodec
from alicia_m_sdk.hardware.device import Device
from alicia_m_sdk.hardware.frame import Frame


class JointResponseDispatchTests(unittest.TestCase):
    def test_write_ack_never_becomes_joint_feedback(self):
        device = Device(None, MessageCodec())
        for result in (1, 0):
            frame = Frame.decode(Frame(0x06, 0x82, bytes([0x80, 2, result])).encode())
            with patch('alicia_m_sdk.hardware.device.logger') as log:
                device._dispatch_response(frame)
                self.assertFalse(log.debug.called, 'A valid ACK must not report a telemetry parse error')
                self.assertEqual(log.warning.called, result == 0)
            self.assertIsNone(device.joint_state)

    def test_interleaved_ack_does_not_replace_or_refresh_telemetry(self):
        device = Device(None, MessageCodec())
        for address_count in (3, 7):
            for start in (0, 0x80):
                data = bytes([start, address_count]) + bytes(7*address_count*2) + bytes([0])
                device._dispatch_response(Frame.decode(Frame(0x06, 0x02, data).encode()))
                state = device.joint_state
                self.assertIsNotNone(state)
                self.assertEqual(len(state.angles), 6)
                device._dispatch_response(Frame(0x06, 0x82, bytes([0x80, 2, 1])))
                self.assertIs(device.joint_state, state)

    def test_truncated_telemetry_is_still_reported(self):
        device = Device(None, MessageCodec())
        with patch('alicia_m_sdk.hardware.device.logger') as log:
            device._dispatch_response(Frame(0x06, 0x02, bytes([0, 2, 0])))
            self.assertTrue(any('解析失败' in str(c) for c in log.debug.call_args_list))
        self.assertIsNone(device.joint_state)


if __name__ == '__main__': unittest.main()
