"""Software-only acceptance checks for the deterministic broker soak."""

from __future__ import annotations

import unittest

from .telemetry_soak import run_soak


class TelemetrySoakTest(unittest.TestCase):
    def test_real_broker_preserves_radio_through_faults_reset_and_slow_consumers(self) -> None:
        result = run_soak(
            cycles=24,
            seed=0x475354,
            reset_interval=19,
            slow_interval=4,
        )

        self.assertTrue(result["ok"], result)
        self.assertTrue(result["radio"]["device_to_host"]["exact"])
        self.assertTrue(result["radio"]["host_to_device"]["exact"])
        self.assertEqual(result["gps"]["drops"], 0)
        self.assertEqual(result["imu"]["drops"], 0)
        self.assertTrue(result["imu"]["exact"])
        self.assertGreaterEqual(result["faults"]["crc"], 1)
        self.assertGreaterEqual(result["faults"]["invalid_escape"], 1)
        self.assertEqual(result["faults"]["resets"], 1)
        self.assertGreaterEqual(result["faults"]["sequence_rollovers"], 1)
        self.assertLess(result["faults"]["max_recovery_ms"], 2000)
        self.assertTrue(result["cleanup"]["broker_thread_stopped"])
        self.assertTrue(result["cleanup"]["runtime_endpoints_removed"])

    def test_rejects_non_soak_parameters(self) -> None:
        with self.assertRaisesRegex(ValueError, "cycles"):
            run_soak(cycles=15)
        with self.assertRaisesRegex(ValueError, "reset_interval"):
            run_soak(cycles=16, reset_interval=7)
        with self.assertRaisesRegex(ValueError, "slow_interval"):
            run_soak(cycles=16, slow_interval=1)


if __name__ == "__main__":
    unittest.main()
