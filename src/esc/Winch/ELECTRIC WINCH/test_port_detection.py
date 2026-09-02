import unittest

from serial_port_utils import find_best_port


class FakePort:
    def __init__(self, device, vid=0, manufacturer="", product=""):
        self.device = device
        self.vid = vid
        self.manufacturer = manufacturer
        self.product = product


class PortDetectionTests(unittest.TestCase):
    def test_prefers_stm32_cdc_port_when_present(self):
        ports = [
            FakePort("COM4", 0x10C4, "Silicon Labs", "CP210x USB UART Bridge"),
            FakePort("COM3", 0x0483, "STMicroelectronics", "STM32 Virtual ComPort"),
        ]
        self.assertEqual(find_best_port(ports), "COM3")

    def test_uses_single_port_when_only_one_exists(self):
        ports = [FakePort("COM5", 0x0483, "STMicroelectronics", "STM32 Virtual ComPort")]
        self.assertEqual(find_best_port(ports), "COM5")

    def test_falls_back_to_first_available_port_when_no_preference_matches(self):
        ports = [
            FakePort("COM7", 0x2341, "Arduino", "Arduino Uno"),
            FakePort("COM8", 0x1A86, "QinHeng Electronics", "USB Serial"),
        ]
        self.assertEqual(find_best_port(ports), "COM7")


if __name__ == "__main__":
    unittest.main()
