"""Reject boot-only or disconnected peripheral evidence for S3 control proof."""
import unittest
from qemu_smoke import control_evidence

class ControlEvidenceTests(unittest.TestCase):
    def test_boot_and_static_fake_screen_are_insufficient(self):
        self.assertFalse(control_evidence("=== Boot complete ===\n")["passed"])
        self.assertFalse(control_evidence('EPPG_QEMU {"state":1,"pixels":5,"screen_fnv1a":42,"throttle_age_ms":5}\nEPPG_QEMU_CAN tx=3 rx=4 pwm_01us=10000')["passed"])

    def test_armed_task_spi_changes_and_nonidle_can_required(self):
        log = '\n'.join([
            'EPPG_QEMU {"state":0,"pixels":10,"screen_fnv1a":42,"throttle_age_ms":5000}',
            'EPPG_QEMU {"state":1,"pixels":20,"screen_fnv1a":43,"throttle_age_ms":9}',
            'EPPG_QEMU_CAN tx=3 rx=4 pwm_01us=15000'])
        self.assertTrue(control_evidence(log)["passed"])
        for old,new in [('"state":1','"state":0'),('"throttle_age_ms":9','"throttle_age_ms":5000'),('"screen_fnv1a":43','"screen_fnv1a":42'),('rx=4','rx=0'),('pwm_01us=15000','pwm_01us=10000')]:
            self.assertFalse(control_evidence(log.replace(old,new))["passed"],old)

if __name__ == '__main__':
    unittest.main()
