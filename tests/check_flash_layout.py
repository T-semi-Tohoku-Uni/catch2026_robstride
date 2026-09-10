"""Check ELF32 load segments against STM32G474RB Flash with DBANK=1.

Usage: python tests/check_flash_layout.py build/Debug/catch2026_robstride.elf
Only Python's standard library is required. Does not connect to hardware.
"""

import struct
import sys
from pathlib import Path

BANKS = ((0x08000000, 0x08010000), (0x08040000, 0x08050000))
RAM = (0x20000000, 0x20020000)


def check(path):
    data = Path(path).read_bytes()
    if len(data) < 52 or data[:6] != b"\x7fELF\x01\x01":
        raise ValueError("Expected a little-endian ELF32 file")
    header = struct.unpack_from("<HHIIIIIHHHHHH", data, 16)
    machine, entry, phoff = header[1], header[3], header[4]
    phentsize, phnum = header[8], header[9]
    if (header[0] != 2 or machine != 40 or phoff < 52 or phentsize < 32
            or phoff + phentsize * phnum > len(data)):
        raise ValueError("Invalid ARM ELF program headers")
    if not entry & 1:
        raise ValueError("Reset entry must select Thumb state")
    if not BANKS[0][0] <= (entry & ~1) < BANKS[0][1]:
        raise ValueError("Reset entry must remain in Bank 1")
    used = [0, 0]
    programmed = []
    vectors = None
    entry_executable = False
    for index in range(phnum):
        kind, offset, virtual, address, size, memory_size, flags, _ = struct.unpack_from(
            "<IIIIIIII", data, phoff + index * phentsize)
        if kind != 1:
            continue
        if size > memory_size:
            raise ValueError("Load segment file size exceeds memory size")
        if memory_size and not any(
                low <= virtual < virtual + memory_size <= high for low, high in (*BANKS, RAM)):
            raise ValueError("Load segment outside runtime memory")
        if size == 0:
            continue
        if offset + size > len(data):
            raise ValueError("Truncated load segment")
        end = address + size
        for bank, (low, high) in enumerate(BANKS):
            if low <= address < end <= high:
                if bank == 1 and (address % 16 or size % 16):
                    raise ValueError("Bank 2 must be 16-byte aligned and padded for CubeProgrammer's two buffers")
                used[bank] += size
                if any(address < previous_end and previous_start < end
                       for previous_start, previous_end in programmed):
                    raise ValueError("Overlapping Flash load segments")
                programmed.append((address, end))
                if address == BANKS[0][0]:
                    if virtual != address or size < 8:
                        raise ValueError("Invalid reset vector segment")
                    vectors = struct.unpack_from("<II", data, offset)
                if flags & 1 and virtual == address and virtual <= (entry & ~1) < virtual + size - 1:
                    entry_executable = True
                print(f"Bank {bank + 1}: 0x{address:08X}..0x{end - 1:08X} ({size} bytes)")
                break
        else:
            raise ValueError(f"Load segment outside Flash banks: 0x{address:08X}..0x{end - 1:08X}")
    if vectors is None or not all(used):
        raise ValueError("Expected vectors in Bank 1 and CyberGear code in Bank 2")
    stack, reset = vectors
    if not RAM[0] < stack <= RAM[1] or stack % 8:
        raise ValueError("Initial stack must be 8-byte aligned in RAM")
    if reset != entry:
        raise ValueError("Reset vector must match the ELF Thumb entry")
    if not entry_executable:
        raise ValueError("Reset entry has no executable Flash bytes")
    print(f"PASS {path}: no programmed bytes in the Flash gap")


if __name__ == "__main__":
    try:
        if len(sys.argv) < 2:
            raise ValueError("Provide one or more firmware ELF paths")
        for filename in sys.argv[1:]:
            check(filename)
    except (ValueError, OSError, struct.error) as error:
        sys.exit(str(error))
