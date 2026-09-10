import contextlib
import io
import struct
import tempfile
import unittest
from pathlib import Path

from check_flash_layout import BANKS, check


class FlashLayoutTests(unittest.TestCase):
    def setUp(self):
        self.entry = BANKS[0][0] + 9
        self.segments = [
            [BANKS[0][0], BANKS[0][0], struct.pack("<II", 0x20020000, self.entry) + bytes(8), 16, 5],
            [0x20000000, BANKS[0][0] + 16, bytes(8), 8, 6],
            [BANKS[1][0], BANKS[1][0], bytes(16), 16, 5],
            [0x20000008, BANKS[0][0] + 24, b"", 32, 6],
        ]

    def firmware(self):
        data = bytearray(0x500)
        data[:16] = b"\x7fELF\x01\x01\x01" + bytes(9)
        struct.pack_into("<HHIIIIIHHHHHH", data, 16,
                         2, 40, 1, self.entry, 52, 0, 0, 52, 32, len(self.segments), 0, 0, 0)
        for index, (virtual, physical, payload, memory_size, flags) in enumerate(self.segments):
            offset = 0x100 * (index + 1)
            struct.pack_into("<IIIIIIII", data, 52 + 32 * index,
                             1, offset, virtual, physical, len(payload), memory_size, flags, 4)
            data[offset:offset + len(payload)] = payload
        return data

    def check_firmware(self, data=None):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "firmware.elf"
            path.write_bytes(self.firmware() if data is None else data)
            with contextlib.redirect_stdout(io.StringIO()):
                check(path)

    def test_valid_firmware_with_initialized_data_and_ram_only_bss(self):
        self.check_firmware()

    def test_flash_gap_is_rejected(self):
        self.segments[1][1] = BANKS[0][1]
        with self.assertRaisesRegex(ValueError, "outside Flash banks"):
            self.check_firmware()

    def test_bank_overflow_is_rejected(self):
        self.segments[1][1] = BANKS[0][1] - 4
        with self.assertRaisesRegex(ValueError, "outside Flash banks"):
            self.check_firmware()

    def test_bank_two_padding_is_required(self):
        self.segments[2][2] = bytes(8)
        self.segments[2][3] = 8
        with self.assertRaisesRegex(ValueError, "16-byte aligned"):
            self.check_firmware()

    def test_truncated_segment_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "Truncated load segment"):
            self.check_firmware(self.firmware()[:0x308])

    def test_overlapping_flash_segments_are_rejected(self):
        self.segments[1][1] = BANKS[0][0] + 8
        with self.assertRaisesRegex(ValueError, "Overlapping Flash"):
            self.check_firmware()

    def test_reset_stack_must_be_aligned_in_ram(self):
        for stack in (0x08008000, 0x20020008, 0x2001FFFF):
            with self.subTest(stack=stack):
                self.segments[0][2] = struct.pack("<II", stack, self.entry) + bytes(8)
                with self.assertRaisesRegex(ValueError, "Initial stack"):
                    self.check_firmware()

    def test_reset_vector_must_match_thumb_entry(self):
        self.segments[0][2] = struct.pack("<II", 0x20020000, self.entry + 2) + bytes(8)
        with self.assertRaisesRegex(ValueError, "Reset vector"):
            self.check_firmware()

    def test_entry_must_be_thumb(self):
        self.entry &= ~1
        with self.assertRaisesRegex(ValueError, "Thumb"):
            self.check_firmware()

    def test_reset_entry_must_have_executable_bytes(self):
        self.entry = BANKS[0][0] + 0x81
        self.segments[0][2] = struct.pack("<II", 0x20020000, self.entry) + bytes(8)
        with self.assertRaisesRegex(ValueError, "executable"):
            self.check_firmware()

    def test_ram_only_segment_must_fit_ram(self):
        self.segments[3][0] = 0x2001FFF0
        with self.assertRaisesRegex(ValueError, "runtime memory"):
            self.check_firmware()

    def test_file_size_must_not_exceed_memory_size(self):
        self.segments[1][3] = 4
        with self.assertRaisesRegex(ValueError, "memory size"):
            self.check_firmware()


if __name__ == "__main__":
    unittest.main()
