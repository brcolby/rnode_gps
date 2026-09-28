import hashlib
import unittest

from Tools.esp_image_hash import validation_hash


class EspImageHashTest(unittest.TestCase):
    def image(self, payload: bytes = b"firmware") -> bytes:
        header = bytearray(24)
        header[23] = 1
        body = bytes(header) + payload
        return body + hashlib.sha256(body).digest()

    def test_returns_valid_appended_hash(self):
        image = self.image()
        self.assertEqual(validation_hash(image), image[-32:])

    def test_rejects_image_without_appended_hash_flag(self):
        image = bytearray(self.image())
        image[23] = 0
        with self.assertRaisesRegex(ValueError, "does not declare"):
            validation_hash(bytes(image))

    def test_rejects_corrupt_image(self):
        image = bytearray(self.image())
        image[24] ^= 0x01
        with self.assertRaisesRegex(ValueError, "invalid appended hash"):
            validation_hash(bytes(image))


if __name__ == "__main__":
    unittest.main()
