#!/usr/bin/env python3
#
# 20260523 dr'Zed of SymbiosiS
# SPDX-License-Identifier: MIT
#
"""ez80dbg — eZ80F91 ZDI Flash Loader and Debugger

Purpose
-------
Flash, load, run, and debug firmware on an eZ80F91 microcontroller via the
Zilog Debug Interface (ZDI).  The ZDI is a two-wire (ZCL/ZDA) protocol spoken
over an FTDI FT4232H adapter in MPSSE mode using pyftdi.

Features
--------
- Write Intel HEX files to internal flash or SRAM
- CPU reset / halt / run / single-step
- Breakpoint: set hardware BP, run, wait for halt, then step
- Register dump with inline eZ80/Z80 disassembly at PC (ADL-aware)
- Hex memory dump
- Tkinter GUI with register panel, runtime disassembly and output log
- eZ80 ADL Mode aware disassembler (ez80dis.py, based on z80dis)

Supported hardware
------------------
Adapter : FTDI FT4232H (default port B, MPSSE )
Target  : eZ80F91 with ZDI pins (ZCL, ZDA) wired to FT4232H Port B:
          ZCL on BDBUS0
          ZDA on BDBUS1 and BDBUS2 (bridged)
          Both lines require Pullup of about 10k to +3v3
Most features might also work with FT232H and every eZ80 based MCU/CPU with ZDI
Only the flash and ram upload are a bit specialized, as they use a custom 
helper binary, that uses i/o and memory addresses specific to eZ80F91. 
It shouldn't be too hard to adapt to other derivatives, though.
The helper is assembled with SLR180 using iz-cpm or other CP/M Runtime. 


CLI usage
---------
  ez80dbg.py flash  <hexfile>  [--run ADDR]
  ez80dbg.py load   <hexfile>  [--run [ADDR]] [--step]
  ez80dbg.py run    <addr>     [--step]
  ez80dbg.py break  <addr>     [--reset] [--timeout SEC]
  ez80dbg.py dump   <addr>     [length]
  ez80dbg.py regs
  ez80dbg.py reset  [--run ADDR]
  ez80dbg.py --gui

Device string (--device DEV)
-----------------------------
  0ABC01B                      serial "0ABC01", port B  (default)
  /2                           first FT4232H found, port B
  ftdi://ftdi:4232h:0ABC01/2   full pyftdi URL (passed through)
"""

import os
import sys
import time
import argparse

try:
    from pyftdi.ftdi import Ftdi
except ImportError:
    print("Error: pyftdi not found.")
    print("  Set up the venv first:  ./setup.sh  (Linux/macOS)")
    print("                          setup.bat   (Windows)")
    sys.exit(1)

try:
    from intelhex import IntelHex
except ImportError:
    print("Error: intelhex not found.  Run setup script first.")
    sys.exit(1)

try:
    import ez80dis
    HAS_DISASM = True
except ImportError:
    HAS_DISASM = False

# ---------------------------------------------------------------------------
# FTDI / ZDI constants
# ---------------------------------------------------------------------------

# Port-B MPSSE low-byte GPIO bit assignments
ZCL         = 0x01   # ZDI clock  (output)
ZDA         = 0x02   # ZDI data   (output)
RESET_PIN   = 0x08   # CPU /RESET (output, high = CPU running)
OUTPUT_MASK = ZCL | ZDA | RESET_PIN

# ZDI write-register addresses
ZDI_ADDR0_L  = 0x00
ZDI_ADDR0_H  = 0x01
ZDI_ADDR0_U  = 0x02
ZDI_ADDR1_L  = 0x04
ZDI_ADDR1_H  = 0x05
ZDI_ADDR1_U  = 0x06
ZDI_ADDR2_L  = 0x08
ZDI_ADDR2_H  = 0x09
ZDI_ADDR2_U  = 0x0A
ZDI_ADDR3_L  = 0x0C
ZDI_ADDR3_H  = 0x0D
ZDI_ADDR3_U  = 0x0E
ZDI_BRK_CTL  = 0x10
ZDI_MSTR_CTL = 0x11
ZDI_WR_DATA_L= 0x13   # WR_DATA_H=0x14, WR_DATA_U=0x15 follow
ZDI_RW_CTL   = 0x16
ZDI_IS4      = 0x21
ZDI_IS3      = 0x22
ZDI_IS2      = 0x23
ZDI_IS1      = 0x24
ZDI_IS0      = 0x25
ZDI_WR_MEM   = 0x30

# ZDI read-register addresses
ZDI_ID_L     = 0x00
ZDI_ID_H     = 0x01
ZDI_ID_REV   = 0x02
ZDI_STAT     = 0x03
ZDI_RD_L     = 0x10   # RD_H=0x11, RD_U=0x12 follow
ZDI_RD_MEM   = 0x20

# ZDI_RW_CTL command codes
RWCTL_READ_MBAF  = 0x00
RWCTL_READ_BC    = 0x01
RWCTL_READ_DE    = 0x02
RWCTL_READ_HL    = 0x03
RWCTL_READ_IX    = 0x04
RWCTL_READ_IY    = 0x05
RWCTL_READ_SP    = 0x06
RWCTL_READ_PC    = 0x07
RWCTL_SET_ADL    = 0x08
RWCTL_RES_ADL    = 0x09
RWCTL_EXCHG      = 0x0A
RWCTL_READ_MEM   = 0x0B
RWCTL_WRITE_MBAF = 0x80
RWCTL_WRITE_BC   = 0x81
RWCTL_WRITE_DE   = 0x82
RWCTL_WRITE_HL   = 0x83
RWCTL_WRITE_IX   = 0x84
RWCTL_WRITE_IY   = 0x85
RWCTL_WRITE_SP   = 0x86
RWCTL_WRITE_PC   = 0x87
RWCTL_WRITE_MEM  = 0x8B

# BRK_CTL operating modes
BRK_RUN    = 0x00   # run freely
BRK_RUN_BP = 0x08   # run, break on BP0
BRK_HALT   = 0x80   # halt immediately
BRK_STEP   = 0x81   # single step

STAT_ZDI     = 0x80   # 1 = CPU in ZDI mode (CPU stopped), 0 = CPU running
STAT_HALTED  = 0x20   # 1 = CPU in HALT instruction or sleep
STAT_ADL     = 0x10   # 1 = ADL mode, 0 = Z80 compatibility mode
STAT_MIXED   = 0x08   # 1 = Mixed Memory Mode active
STAT_IEF     = 0x04   # 1 = Maskable interrupts enabled
EZ80_CHIP_ID = 0x08   # expected ZDI_ID_L

HALT_TIMEOUT_DEFAULT = 10.0


# ---------------------------------------------------------------------------
# Device URL helper
# ---------------------------------------------------------------------------

def _build_url(device_str: str) -> str:
    """Convert a device string to a pyftdi URL.

    Accepted formats:
      '0ABC01B'                 serial + port letter  -> ftdi://ftdi:4232h:0ABC01/2
      '0ABC01'                  serial only           -> ftdi://ftdi:4232h:0ABC01/2
      '/2'                      first device, port B  -> ftdi://ftdi:4232h/2
      'ftdi://...'              full URL, passed through unchanged
    """
    if device_str.lower().startswith('ftdi://'):
        return device_str
    port_map = {'A': 1, 'B': 2, 'C': 3, 'D': 4}
    s = device_str.strip()
    if s and s[-1].upper() in port_map:
        iface  = port_map[s[-1].upper()]
        serial = s[:-1]
    else:
        iface  = 2
        serial = s
    if serial:
        return f'ftdi://ftdi:4232h:{serial}/{iface}'
    return f'ftdi://ftdi:4232h/{iface}'


# ---------------------------------------------------------------------------
# ZDIDebugger
# ---------------------------------------------------------------------------

class ZDIDebugger:
    """ZDI protocol over FTDI FT4232H MPSSE (bit-bang GPIO, Port B)."""

    def __init__(self, device='0ABC01B'):
        self._url  = _build_url(str(device))
        self._ftdi = None
        self._buf  = []

    # ---- connection --------------------------------------------------------

    def open(self):
        self._ftdi = Ftdi()
        self._ftdi.open_mpsse_from_url(
            self._url,
            frequency = 2e6,
            direction = OUTPUT_MASK,
            initial   = RESET_PIN | ZCL | ZDA,
            latency   = 1,
        )
        self._ftdi.purge_buffers()
        print(f"Opened: {self._url}")
        chip_id = self._zdi_read_reg(ZDI_ID_L)
        if chip_id != EZ80_CHIP_ID:
            raise RuntimeError(
                f"Chip ID mismatch: expected 0x{EZ80_CHIP_ID:02X}, got 0x{chip_id:02X}"
            )
        print(f"eZ80F91 detected  (ID=0x{chip_id:02X}, "
              f"STAT=0x{self._zdi_read_reg(ZDI_STAT):02X})")
        return self

    def close(self):
        if self._ftdi:
            try:
                self._ftdi.close()
            except Exception:
                pass
            self._ftdi = None

    def __enter__(self): return self.open()
    def __exit__(self, *_): self.close()

    # ---- raw transport -----------------------------------------------------

    def _ft_write(self, data):
        self._ftdi.write_data(bytes(data))

    def _ft_read(self, n: int) -> list:
        return list(self._ftdi.read_data_bytes(n, attempt=5))

    # ---- ZDI bit-bang protocol --------------------------------------------
    # 2-wire serial protocol: ZCL = clock, ZDA = data, MSB first.
    # Each transaction starts with a START condition; bytes are separated by
    # a Single-Bit Byte Separator (SBBS) pulse.
    # All line state is set via MPSSE command 0x80 (Set GPIO Low Byte).

    def _codebyte(self, byte: int):
        """Append MPSSE commands to clock out one ZDI data byte + SBBS."""
        for i in range(7, -1, -1):
            bit = ZDA if (byte & (1 << i)) else 0
            self._buf += [0x80, RESET_PIN | bit,         OUTPUT_MASK]
            self._buf += [0x80, RESET_PIN | ZCL | bit,   OUTPUT_MASK]
            self._buf += [0x80, RESET_PIN | bit,         OUTPUT_MASK]
        # SBBS: ZDA high while ZCL pulses once
        self._buf += [0x80, RESET_PIN | ZDA,         OUTPUT_MASK]
        self._buf += [0x80, RESET_PIN | ZCL | ZDA,   ZCL | RESET_PIN]

    def _zdi_start(self):
        """ZDI START: ZDA falls while ZCL is high, then ZCL falls."""
        self._buf = []
        b = self._buf
        b += [0x80, ZCL | ZDA | RESET_PIN, OUTPUT_MASK]
        b += [0x80, ZCL | ZDA | RESET_PIN, OUTPUT_MASK]
        b += [0x80, ZCL | ZDA | RESET_PIN, OUTPUT_MASK]
        b += [0x80, ZCL | RESET_PIN,       OUTPUT_MASK]
        b += [0x80, ZCL | RESET_PIN,       OUTPUT_MASK]
        b += [0x80, ZCL | RESET_PIN,       OUTPUT_MASK]
        b += [0x80, RESET_PIN,             OUTPUT_MASK]
        b += [0x80, RESET_PIN,             OUTPUT_MASK]
        b += [0x80, RESET_PIN,             OUTPUT_MASK]

    def _zdi_write_reg(self, addr: int, value: int):
        self._zdi_start()
        self._codebyte((addr & 0x7F) << 1)
        self._codebyte(value & 0xFF)
        self._ft_write(self._buf)

    def _zdi_write_multiple(self, start_addr: int, data):
        """Write consecutive ZDI registers in one transaction."""
        self._zdi_start()
        self._codebyte((start_addr & 0x7F) << 1)
        for b in data:
            self._codebyte(b & 0xFF)
        self._ft_write(self._buf)

    def _zdi_read_reg(self, addr: int) -> int:
        """Read one ZDI register (no extra clock-prep byte)."""
        self._zdi_start()
        self._codebyte(((addr & 0x7F) << 1) | 1)
        self._buf += [0x20, 0, 0]                           # MPSSE: read 1 byte
        self._buf += [0x80, RESET_PIN,        ZCL | RESET_PIN]
        self._buf += [0x80, ZCL | RESET_PIN,  ZCL | RESET_PIN]
        self._buf += [0x87]                                 # flush USB buffer
        self._ft_write(self._buf)
        return self._ft_read(1)[0]

    def _zdi_read_multiple(self, start_addr: int, count: int) -> list:
        """Read count consecutive ZDI registers in one transaction."""
        self._zdi_start()
        self._codebyte(((start_addr & 0x7F) << 1) | 1)
        self._buf += [0x80, ZCL | RESET_PIN, ZCL | RESET_PIN]   # clock prep
        for _ in range(count):
            self._buf += [0x20, 0, 0]
            self._buf += [0x80, RESET_PIN,        ZCL | RESET_PIN]
            self._buf += [0x80, ZCL | RESET_PIN,  ZCL | RESET_PIN]
        self._buf += [0x87]
        self._ft_write(self._buf)
        return self._ft_read(count)

    # ---- CPU control -------------------------------------------------------

    def cpu_halt(self):    self._zdi_write_reg(ZDI_BRK_CTL, BRK_HALT)
    def cpu_run(self):     self._zdi_write_reg(ZDI_BRK_CTL, BRK_RUN)
    def cpu_step(self):    self._zdi_write_reg(ZDI_BRK_CTL, BRK_STEP)
    def cpu_run_to_bp(self): self._zdi_write_reg(ZDI_BRK_CTL, BRK_RUN_BP)


    def read_status(self) -> int:
        return self._zdi_read_reg(ZDI_STAT)

    def is_halted(self) -> bool:
        return bool(self.read_status() & STAT_ZDI)

    def is_adl(self) -> bool:
        return bool(self.read_status() & STAT_ADL)

    def wait_halt(self, timeout: float = HALT_TIMEOUT_DEFAULT) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.is_halted():
                return True
        return False

    def set_breakpoint(self, bp_num: int, addr: int):
        base = (ZDI_ADDR0_L, ZDI_ADDR1_L, ZDI_ADDR2_L, ZDI_ADDR3_L)[bp_num]
        self._zdi_write_multiple(base, [
            addr & 0xFF, (addr >> 8) & 0xFF, (addr >> 16) & 0xFF
        ])

    # ---- CPU init after reset ---------------------------------------------

    def cpu_reset(self):
        """Reset CPU: MBASE=0, ADL=1, RAM/Flash at page 0, DI."""
        print("Resetting...")
        self._zdi_write_reg(ZDI_MSTR_CTL, 0x80)        # trigger chip reset
        self._zdi_write_reg(ZDI_BRK_CTL, BRK_HALT)
        if not self.wait_halt(timeout=2.0):
            raise RuntimeError("CPU did not halt after reset")   
        self._zdi_write_multiple(ZDI_WR_DATA_L, [0, 0, 0, RWCTL_WRITE_MBAF]) # clear A and MBASE
        self._zdi_write_multiple(ZDI_IS0, [0xF3])                    # DI
        self._zdi_write_multiple(ZDI_IS2, [0xB5, 0x39, 0xED])        # OUT0(RAM_ADDR_U), a
        self._zdi_write_multiple(ZDI_IS2, [0xF7, 0x39, 0xED])        # OUT0(FLASH_ADDR_U), a
        self._zdi_write_reg(ZDI_RW_CTL, RWCTL_SET_ADL)
        self._zdi_write_multiple(ZDI_WR_DATA_L, [0, 0, 0, RWCTL_WRITE_PC]) # set PC to 0
        if not self.is_halted():
            raise RuntimeError("CPU did not halt after init sequence")
        else:
            print(f"Halted @{self.read_pc()}")

    def cpu_init_for_helper(self):
        """Reset CPU: MBASE=0, ADL=0, RAM/Flash at page 0, DI."""
        print("Resetting...")
        self._zdi_write_reg(ZDI_MSTR_CTL, 0x80)        # trigger chip reset
        self._zdi_write_reg(ZDI_BRK_CTL, BRK_HALT)
        if not self.wait_halt(timeout=2.0):
            raise RuntimeError("CPU did not halt after reset")   
        self._zdi_write_multiple(ZDI_WR_DATA_L, [0, 0, 0, RWCTL_WRITE_MBAF]) # clear A and MBASE
        self._zdi_write_multiple(ZDI_IS0, [0xF3])                    # DI
        self._zdi_write_multiple(ZDI_IS2, [0xB5, 0x39, 0xED])        # OUT0(RAM_ADDR_U), a
        self._zdi_write_multiple(ZDI_IS2, [0xF7, 0x39, 0xED])        # OUT0(FLASH_ADDR_U), a
        self._zdi_write_reg(ZDI_RW_CTL, RWCTL_RES_ADL)
        self._zdi_write_multiple(ZDI_WR_DATA_L, [0, 0, 0, RWCTL_WRITE_PC]) # set PC to 0
        if not self.is_halted():
            raise RuntimeError("CPU did not halt after init sequence")
        else:
            print(f"Halted @{self.read_pc()}")

    # ---- Register / memory access -----------------------------------------

    def write_pc(self, addr: int):
        was_adl = self.is_adl()
        if not was_adl:
            self._zdi_write_reg(ZDI_RW_CTL, RWCTL_SET_ADL)
        self._zdi_write_multiple(ZDI_WR_DATA_L, [
            addr & 0xFF, (addr >> 8) & 0xFF, (addr >> 16) & 0xFF, RWCTL_WRITE_PC
        ])
        if not was_adl:
            self._zdi_write_reg(ZDI_RW_CTL, RWCTL_RES_ADL)

    def read_pc(self) -> int:
        self._zdi_write_reg(ZDI_RW_CTL, RWCTL_READ_PC)
        b = self._zdi_read_multiple(ZDI_RD_L, 3)
        return b[0] | (b[1] << 8) | (b[2] << 16)

    def read_all_regs(self) -> dict:
        r = {}

        orig_adl = bool(self._zdi_read_reg(ZDI_STAT) & STAT_ADL)
        if not orig_adl:
            self._zdi_write_reg(ZDI_RW_CTL, RWCTL_SET_ADL)

        # Primary registers (SP handled separately)
        for cmd, keys in (
            (RWCTL_READ_MBAF, ('A',   'F',   'MBASE')),
            (RWCTL_READ_BC,   ('C',   'B',   'BCU'  )),
            (RWCTL_READ_DE,   ('E',   'D',   'DEU'  )),
            (RWCTL_READ_HL,   ('L',   'H',   'HLU'  )),
            (RWCTL_READ_IX,   ('IXL', 'IXH', 'IXU'  )),
            (RWCTL_READ_IY,   ('IYL', 'IYH', 'IYU'  )),
            (RWCTL_READ_PC,   ('PCL', 'PCH', 'PCU'  )),
        ):
            self._zdi_write_reg(ZDI_RW_CTL, cmd)
            vals = self._zdi_read_multiple(ZDI_RD_L, 3)
            for k, v in zip(keys, vals):
                r[k] = v

        # READ_SP in ADL mode → SPL; toggle to Z80 → SPS; toggle back.
        self._zdi_write_reg(ZDI_RW_CTL, RWCTL_READ_SP)
        b = self._zdi_read_multiple(ZDI_RD_L, 3)
        r['SPL_L'], r['SPL_H'], r['SPL_U'] = b[0], b[1], b[2]
        self._zdi_write_reg(ZDI_RW_CTL, RWCTL_RES_ADL)
        self._zdi_write_reg(ZDI_RW_CTL, RWCTL_READ_SP)
        b2 = self._zdi_read_multiple(ZDI_RD_L, 3)
        r['SPS_L'], r['SPS_H'] = b2[0], b2[1]
        self._zdi_write_reg(ZDI_RW_CTL, RWCTL_SET_ADL)

        # Second register bank: EXCHG swaps primary ↔ shadow, read, swap back.
        self._zdi_write_reg(ZDI_RW_CTL, RWCTL_EXCHG)
        for cmd, keys in (
            (RWCTL_READ_MBAF, ('pA', 'pF', None    )),
            (RWCTL_READ_BC,   ('pC', 'pB', 'pBCU'  )),
            (RWCTL_READ_DE,   ('pE', 'pD', 'pDEU'  )),
            (RWCTL_READ_HL,   ('pL', 'pH', 'pHLU'  )),
        ):
            self._zdi_write_reg(ZDI_RW_CTL, cmd)
            vals = self._zdi_read_multiple(ZDI_RD_L, 3)
            for k, v in zip(keys, vals):
                if k is not None:
                    r[k] = v
        self._zdi_write_reg(ZDI_RW_CTL, RWCTL_EXCHG)

        if not orig_adl:
            self._zdi_write_reg(ZDI_RW_CTL, RWCTL_RES_ADL)

        return r

    _READ_CHUNK = 256

    def read_mem(self, addr: int, count: int, progress=None) -> list:
        """Read count bytes from addr in chunks. CPU must be halted.
        progress: optional callable invoked after each chunk (e.g. for dots)."""
        was_adl = self.is_adl()
        if not was_adl:
            self._zdi_write_reg(ZDI_RW_CTL, RWCTL_SET_ADL)
        result = []
        remaining = count
        offset = 0
        while remaining:
            n = min(remaining, self._READ_CHUNK)
            self._zdi_write_multiple(ZDI_WR_DATA_L, [
                (addr + offset) & 0xFF, ((addr + offset) >> 8) & 0xFF,
                ((addr + offset) >> 16) & 0xFF, RWCTL_WRITE_PC
            ])
            result += self._zdi_read_multiple(ZDI_RD_MEM, n)
            remaining -= n
            offset += n
            if progress:
                progress()
        if not was_adl:
            self._zdi_write_reg(ZDI_RW_CTL, RWCTL_RES_ADL)
        return result

    def write_mem(self, addr: int, data):
        """Write bytes to addr via ZDI auto-increment sequential write."""
        was_adl = self.is_adl()
        if not was_adl:
            self._zdi_write_reg(ZDI_RW_CTL, RWCTL_SET_ADL)
        self._zdi_write_multiple(ZDI_WR_DATA_L, [
            addr & 0xFF, (addr >> 8) & 0xFF, (addr >> 16) & 0xFF, RWCTL_WRITE_PC
        ])
        self._zdi_write_multiple(ZDI_WR_MEM, data)
        if not was_adl:
            self._zdi_write_reg(ZDI_RW_CTL, RWCTL_RES_ADL)

    # ---- Flash programming ------------------------------------------------

    def flash_file(self, hexfile_path: str, helper_path: str = None):
        """Program hex file to internal Flash using the ZDIHELP helper blob."""
        if helper_path is None:
            helper_path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)), 'ZDIHELP.HEX'
            )

        print(f"Helper:  {helper_path}")
        helper = IntelHex()
        helper.fromfile(helper_path, format='hex')
        helper.padding = 0x00
        h_bin = helper.tobinstr(helper.minaddr(), helper.maxaddr())

        magic = h_bin[3] | (h_bin[4] << 8)
        if magic != 0xAA55:
            raise RuntimeError(f"Helper magic wrong: 0x{magic:04X} (expected 0xAA55)")
        park_pos = h_bin[5] | (h_bin[6] << 8)
        data_pos = h_bin[7] | (h_bin[8] << 8)
        exec_pos = park_pos + 3
        print(f"         park=0x{park_pos:04X}  data=0x{data_pos:04X}  exec=0x{exec_pos:04X}")

        print(f"Target:  {hexfile_path}")
        target = IntelHex()
        target.fromfile(hexfile_path, format='hex')
        target.padding = 0x00
        t_bin = target.tobinstr(target.minaddr(), target.maxaddr())
        print(f"         0x{target.minaddr():06X}-0x{target.maxaddr():06X}  ({len(t_bin)} bytes)")

        print("Initialising CPU...")
        self.cpu_init_for_helper()
        self.set_breakpoint(0, park_pos)

        print("Uploading helper to SRAM...", end=' ', flush=True)
        self.write_mem(helper.minaddr(), h_bin)
        self.write_pc(helper.minaddr())
        self.cpu_run_to_bp()
        if not self.wait_halt(timeout=2.0):
            raise RuntimeError("Helper did not reach park position")
        print("OK")

        pages = (len(t_bin) + 2047) // 2048
        print(f"Erasing {pages} page(s)...", end='', flush=True)
        for page in range(pages):
            self._zdi_write_multiple(ZDI_WR_DATA_L, [0, page & 0x7F, 0])
            self._zdi_write_reg(ZDI_RW_CTL, RWCTL_WRITE_HL)
            self._zdi_write_multiple(ZDI_WR_DATA_L, [0xAA, 0, 0])
            self._zdi_write_reg(ZDI_RW_CTL, RWCTL_WRITE_MBAF)
            self.write_pc(exec_pos)
            self.cpu_run_to_bp()
            if not self.wait_halt(timeout=5.0):
                raise RuntimeError(f"Erase timeout at page {page}")
            self._zdi_write_reg(ZDI_RW_CTL, RWCTL_READ_MBAF)
            result = self._zdi_read_multiple(ZDI_RD_L, 3)
            if result[0] != 0x00:
                raise RuntimeError(f"Erase error page {page}: code 0x{result[0]:02X}")
            print('.', end='', flush=True)
        print('  Done')

        rows = (len(t_bin) + 255) // 256
        print(f"Programming {rows} row(s)...", end='', flush=True)
        for row in range(rows):
            chunk = t_bin[row * 256 : row * 256 + 256]
            chunk = chunk.ljust(256, b'\x00')
            self.write_mem(data_pos, chunk)
            page_num = (row * 256) // 2048
            row_num  = row & 0x07
            self._zdi_write_multiple(ZDI_WR_DATA_L, [row_num, page_num, 0])
            self._zdi_write_reg(ZDI_RW_CTL, RWCTL_WRITE_HL)
            self._zdi_write_multiple(ZDI_WR_DATA_L, [0xC3, 0, 0])
            self._zdi_write_reg(ZDI_RW_CTL, RWCTL_WRITE_MBAF)
            self.write_pc(exec_pos)
            self.cpu_run_to_bp()
            if not self.wait_halt(timeout=5.0):
                raise RuntimeError(f"Program timeout at row {row}")
            self._zdi_write_reg(ZDI_RW_CTL, RWCTL_READ_MBAF)
            result = self._zdi_read_multiple(ZDI_RD_L, 3)
            if result[0] != 0x00:
                raise RuntimeError(f"Program error row {row}: code 0x{result[0]:02X}")
            print('.', end='', flush=True)
        print('  Done')
        print("Flash programming complete.")

    # ---- SRAM load --------------------------------------------------------

    def load_sram(self, hexfile_path: str):
        """Load hex file directly to SRAM.  Returns (minaddr, maxaddr)."""
        target = IntelHex()
        target.fromfile(hexfile_path, format='hex')
        target.padding = 0x00
        t_bin = target.tobinstr(target.minaddr(), target.maxaddr())
        mn, mx = target.minaddr(), target.maxaddr()
        if mn < 0xC000 or mx > 0xFFFF:
            raise ValueError(
                f"SRAM load: 0x{mn:04X}-0x{mx:04X} outside valid window 0xC000-0xFFFF"
            )
        print(f"Loading to SRAM 0x{mn:04X}-0x{mx:04X}  ({len(t_bin)} bytes)...")
        self.write_mem(mn, t_bin)
        print("Load complete.")
        return mn, mx


# ---------------------------------------------------------------------------
# Display helpers
# ---------------------------------------------------------------------------

def _flags_str(f: int) -> str:
    return ''.join([
        'S' if f & 0x80 else 's', 'Z' if f & 0x40 else 'z',
        'H' if f & 0x10 else 'h', 'V' if f & 0x04 else 'v',
        'N' if f & 0x02 else 'n', 'C' if f & 0x01 else 'c',
    ])

def phys_pc(regs: dict, adl: bool) -> int:
    upper = regs['PCU'] if adl else regs['MBASE']
    return regs['PCL'] | (regs['PCH'] << 8) | (upper << 16)

def print_regs(regs: dict, adl: bool = False):
    pc  = phys_pc(regs, adl)
    spl = regs['SPL_L'] | (regs['SPL_H'] << 8) | (regs['SPL_U'] << 16)
    sps = regs['SPS_L'] | (regs['SPS_H'] << 8)
    bc  = (regs['BCU'] << 16) | (regs['B']   << 8) | regs['C']
    de  = (regs['DEU'] << 16) | (regs['D']   << 8) | regs['E']
    hl  = (regs['HLU'] << 16) | (regs['H']   << 8) | regs['L']
    ix  = (regs['IXU'] << 16) | (regs['IXH'] << 8) | regs['IXL']
    iy  = (regs['IYU'] << 16) | (regs['IYH'] << 8) | regs['IYL']
    mode = "ADL" if adl else "Z80"
    print(f"  PC={pc:06X}  [{mode}]  MBASE={regs['MBASE']:02X}")
    print(f"  SPS={sps:04X}  SPL={spl:06X}")
    print(f"  Flags: [{_flags_str(regs['F'])}]")
    print(f"  A={regs['A']:02X}")
    print(f"  BC={bc:06X}")
    print(f"  DE={de:06X}")
    print(f"  HL={hl:06X}")
    print(f"  IX={ix:06X}")
    print(f"  IY={iy:06X}")
    if 'pA' in regs:
        pbc = (regs['pBCU'] << 16) | (regs['pB'] << 8) | regs['pC']
        pde = (regs['pDEU'] << 16) | (regs['pD'] << 8) | regs['pE']
        phl = (regs['pHLU'] << 16) | (regs['pH'] << 8) | regs['pL']
        print(f"  Flags' : [{_flags_str(regs['pF'])}]  ")
        print(f"  A'={regs['pA']:02X}")
        print(f"  BC'={pbc:06X}")
        print(f"  DE'={pde:06X}")  
        print(f"  HL'={phl:06X}")

def raw_pc(regs: dict) -> int:
    return regs['PCL'] | (regs['PCH'] << 8) | (regs['PCU'] << 16)

def print_pc_insn(dbg: ZDIDebugger, regs: dict):
    """Print bytes + optional disassembly at current PC, then restore PC."""
    adl = dbg.is_adl()
    pc  = phys_pc(regs, adl)
    mem = dbg.read_mem(pc, 8)
    dbg.write_pc(raw_pc(regs))
    h = ' '.join(f'{b:02X}' for b in mem)
    disasm = ''
    if HAS_DISASM:
        try:
            dec    = ez80dis.decode(mem, pc, adl)
            disasm = f'  =>  {ez80dis.disasm(dec)}'
        except Exception:
            pass
    print(f"  [{pc:06X}]: {h}{disasm}")

def hex_dump(data, start_addr: int = 0, width: int = 16):
    for i in range(0, len(data), width):
        chunk = data[i:i+width]
        h = ' '.join(f'{b:02X}' for b in chunk)
        a = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
        print(f"  {start_addr+i:06X}:  {h:<{width*3}}  {a}")


# ---------------------------------------------------------------------------
# Interactive single-step shell
# ---------------------------------------------------------------------------

STEP_HELP = """\
  <Enter>         single step
  r               run freely (exit stepper)
  b <addr>        run to breakpoint      e.g.  b E123
  m <addr> [len]  hex dump memory        e.g.  m C000 80
  q               quit"""

def interactive_step(dbg: ZDIDebugger):
    print(f"\nSingle-step mode.\n{STEP_HELP}\n")
    while True:
        if not dbg.is_halted():
            print("  CPU not halted - halting now.")
            dbg.cpu_halt()
            if not dbg.wait_halt(timeout=2.0):
                print("  Could not halt CPU.")
                break

        regs = dbg.read_all_regs()
        adl  = dbg.is_adl()
        pc   = phys_pc(regs, adl)
        print_regs(regs, adl)
        print_pc_insn(dbg, regs)

        try:
            line = input(f"  [{pc:06X}]> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not line:
            dbg.cpu_step()
            if not dbg.wait_halt(HALT_TIMEOUT_DEFAULT):
                print("  CPU did not halt after step (timeout).")
                break

        elif line.lower() == 'q':
            break

        elif line.lower() == 'r':
            print("  Running freely.")
            dbg.cpu_run()
            break

        elif line.lower().startswith('b '):
            try:
                addr = int(line[2:].strip(), 16)
                dbg.set_breakpoint(0, addr)
                print(f"  Running to 0x{addr:06X} ...")
                dbg.cpu_run_to_bp()
                if not dbg.wait_halt(HALT_TIMEOUT_DEFAULT):
                    print("  Breakpoint not reached (timeout).")
                    break
                print("  Halted at breakpoint.")
            except ValueError:
                print("  Bad address - hex without 0x, e.g.:  b C000")

        elif line.lower().startswith('m '):
            parts = line[2:].split()
            try:
                addr   = int(parts[0], 16)
                length = int(parts[1], 16) if len(parts) > 1 else 0x40
                data   = dbg.read_mem(addr, length)
                dbg.write_pc(raw_pc(regs))
                hex_dump(data, addr)
            except (ValueError, IndexError):
                print("  Usage:  m <addr_hex> [len_hex]")

        else:
            print(STEP_HELP)


# ---------------------------------------------------------------------------
# CLI command implementations
# ---------------------------------------------------------------------------

def _auto_int(x):
    return int(x, 0)

def cmd_flash(dbg, args):
    """Write hexfile to internal flash via a helper blob loaded into RAM.
    --run ADDR: start execution at ADDR after flashing."""
    dbg.flash_file(args.hexfile)
    if args.run is not None:
        print(f"Running from 0x{args.run:06X}")
        dbg.write_pc(args.run)
        dbg.cpu_run()

def cmd_load(dbg, args):
    """Reset CPU and load hexfile into SRAM.
    --run [ADDR]: start execution after load; defaults to the lowest address in the hex file.
    --step: enter interactive single-step mode instead of free run."""
    dbg.cpu_reset()
    mn, _ = dbg.load_sram(args.hexfile)
    if args.run is not None:
        addr = mn if args.run < 0 else args.run
        print(f"Running from 0x{addr:04X}")
        dbg.write_pc(addr)
        if args.step:
            interactive_step(dbg)
        else:
            dbg.cpu_run()

def cmd_run(dbg, args):
    """Set PC to addr and start execution.
    --step: enter interactive single-step mode instead of free run."""
    dbg.write_pc(args.addr)
    if args.step:
        interactive_step(dbg)
    else:
        print(f"Running from 0x{args.addr:06X}")
        dbg.cpu_run()

def cmd_break(dbg, args):
    """Set BP0 at addr, run the CPU, and wait for the breakpoint to fire.
    --reset: reset the CPU before running.
    --timeout SEC: maximum wait time in seconds (default: HALT_TIMEOUT_DEFAULT).
    Enters interactive single-step mode automatically after halt."""
    if args.reset:
        dbg.cpu_reset()
    dbg.set_breakpoint(0, args.addr)
    print(f"Breakpoint @ 0x{args.addr:06X}, running...")
    dbg.cpu_run_to_bp()
    if not dbg.wait_halt(timeout=args.timeout):
        print("Breakpoint not reached (timeout).")
        return
    pc = dbg.read_pc()
    print(f"Halted @ 0x{pc:06X}")
    interactive_step(dbg)

def cmd_dump(dbg, args):
    """Read length bytes starting at addr and print a hex dump.
    Temporarily clobbers PC (ZDI read mechanism) and restores it afterwards.
    length: byte count, default 0x100."""
    saved_pc = dbg.read_pc()
    data = dbg.read_mem(args.addr, args.length)
    dbg.write_pc(saved_pc)
    hex_dump(data, args.addr)

def cmd_regs(dbg, args):
    """Read and display all CPU registers. Halts the CPU first if it is still running.
    Also disassembles and prints the instruction at the current PC."""
    if not dbg.is_halted():
        print("Halting CPU...")
        dbg.cpu_halt()
        dbg.wait_halt(timeout=2.0)
    regs = dbg.read_all_regs()
    print_regs(regs, dbg.is_adl())
    print_pc_insn(dbg, regs)

def cmd_reset(dbg, args):
    """Reset and halt the CPU, then display registers.
    --run ADDR: start execution at ADDR after reset."""
    dbg.cpu_reset()
    regs = dbg.read_all_regs()
    print_regs(regs, dbg.is_adl())
    if args.run is not None:
        print(f"Running from 0x{args.run:06X}")
        dbg.write_pc(args.run)
        dbg.cpu_run()


# ---------------------------------------------------------------------------
# Argument parser
# ---------------------------------------------------------------------------

class _HelpFormatter(argparse.RawDescriptionHelpFormatter):
    """Three-column subcommand listing: name | synopsis | description."""

    @staticmethod
    def _synopsis(subp):
        """Build 'positionals first, optionals after' synopsis for subparser."""
        pos, opt = [], []
        for act in subp._actions:
            if isinstance(act, argparse._HelpAction):
                continue
            if act.option_strings:
                flag = act.option_strings[-1]
                if isinstance(act, (argparse._StoreTrueAction,
                                    argparse._StoreFalseAction)):
                    opt.append(f'[{flag}]')
                elif act.nargs == '?':
                    mv = act.metavar or act.dest.upper()
                    opt.append(f'[{flag} [{mv}]]')
                else:
                    mv = act.metavar or act.dest.upper()
                    opt.append(f'[{flag} {mv}]')
            else:
                mv = act.metavar or act.dest
                if act.nargs == '?':
                    pos.append(f'[<{mv}>]')
                elif act.nargs == '*':
                    pos.append(f'[<{mv}> ...]')
                elif act.nargs == '+':
                    pos.append(f'<{mv}> [...]')
                else:
                    pos.append(f'<{mv}>')
        return ' '.join(pos + opt)

    def _format_action(self, action):
        if not isinstance(action, argparse._SubParsersAction):
            return super()._format_action(action)
        help_map = {ca.dest: ca.help or '' for ca in action._choices_actions}
        rows = [(name, self._synopsis(subp), help_map.get(name, ''))
                for name, subp in action.choices.items()]
        name_w = max(len(n) for n, _, _ in rows) + 2
        syn_w  = max(len(s) for _, s, _ in rows) + 2
        return ''.join(f'  {n:<{name_w}}{s:<{syn_w}}{d}\n'
                       for n, s, d in rows)


def build_parser():
    p = argparse.ArgumentParser(
        prog='ez80dbg',
        description='eZ80F91 ZDI Flash Loader / Debugger  (transport: pyftdi)',
        formatter_class=_HelpFormatter,
        epilog="""\
device string examples:
  0ABC01B                    serial "0ABC01", port B  (default)
  0ABC01A                    serial "0ABC01", port A
  /2                         first FT4232H found, port B
  ftdi://ftdi:4232h:0ABC01/2 full pyftdi URL""",
    )
    p.add_argument('--device', default='0ABC01B', metavar='DEV',
                   help='FTDI device serial + port letter (default: 0ABC01B)')
    p.add_argument('--gui', action='store_true', help='Launch GUI')

    sub = p.add_subparsers(dest='command',
                           required='--gui' not in sys.argv)

    s = sub.add_parser('flash', help='Write hex file to internal flash')
    s.add_argument('hexfile')
    s.add_argument('--run', type=_auto_int, default=None, metavar='ADDR',
                   help='Start execution after flash')

    s = sub.add_parser('load', help='Load hex file into SRAM')
    s.add_argument('hexfile')
    s.add_argument('--run', type=_auto_int, default=None, const=-1, nargs='?',
                   metavar='ADDR',
                   help='Run after load (at ADDR, or file start if omitted)')
    s.add_argument('--step', action='store_true', help='Single-step mode')

    s = sub.add_parser('run', help='Start execution at address')
    s.add_argument('addr', type=_auto_int, help='Start address (hex OK: 0xC000)')
    s.add_argument('--step', action='store_true', help='Single-step mode')

    s = sub.add_parser('break', help='Set breakpoint, run, wait, then step')
    s.add_argument('addr', type=_auto_int, help='Breakpoint address')
    s.add_argument('--reset', action='store_true', help='Reset CPU first')
    s.add_argument('--timeout', type=float, default=HALT_TIMEOUT_DEFAULT,
                   metavar='SEC', help='Wait timeout in seconds')

    s = sub.add_parser('dump', help='Hex dump a memory region')
    s.add_argument('addr',   type=_auto_int, help='Start address')
    s.add_argument('length', type=_auto_int, nargs='?', default=0x100,
                   help='Byte count (default: 0x100)')

    sub.add_parser('regs', help='Show CPU registers + disassembly at PC')

    s = sub.add_parser('reset', help='Reset and halt CPU')
    s.add_argument('--run', type=_auto_int, default=None, metavar='ADDR',
                   help='Run from ADDR after reset')



    return p


# ---------------------------------------------------------------------------
# Tkinter GUI
# ---------------------------------------------------------------------------

def launch_gui(device: str):
    import tkinter as tk
    from tkinter import ttk, filedialog, scrolledtext, messagebox
    import io, contextlib, threading, queue as _queue

    class App:
        def __init__(self, root: tk.Tk):
            self.root       = root
            self.dbg        = None
            self._device    = device
            self._prev_regs = None
            root.title("eZ80F91 ZDI Debugger")
            root.minsize(860, 520)
            self._build_ui()

        def _build_ui(self):
            tb = ttk.Frame(self.root, padding=(4, 2))
            tb.pack(fill='x', side='top')
            self._build_toolbar(tb)

            self.status_var = tk.StringVar(value="Not connected")
            ttk.Label(self.root, textvariable=self.status_var,
                      relief='sunken', anchor='w',
                      padding=(4, 2)).pack(fill='x', side='bottom')

            pw = ttk.PanedWindow(self.root, orient='horizontal')
            pw.pack(fill='both', expand=True, padx=4, pady=4)
            left  = ttk.Frame(pw, width=270)
            right = ttk.Frame(pw)
            pw.add(left,  weight=0)
            pw.add(right, weight=1)
            self._build_left(left)
            self._build_right(right)

        def _build_toolbar(self, tb):
            groups = [
                [("Connect",    self._connect),
                 ("Disconnect", self._disconnect)],
                [("Reset/Halt", self._reset),
                 ("Halt",      self._halt),
                 ("Run",    self._run),
                 ("Step",   self._step)],
                [("Flash...",    self._flash_dialog),
                 ("Load SRAM...",self._load_dialog)],
            ]
            for group in groups:
                ttk.Separator(tb, orient='vertical').pack(side='left', fill='y', padx=3)
                for label, cmd in group:
                    ttk.Button(tb, text=label, command=cmd, width=10).pack(
                        side='left', padx=1, pady=1)

        def _build_left(self, parent):
            rf = ttk.LabelFrame(parent, text="Registers", padding=4)
            rf.pack(fill='x', padx=4, pady=(4, 2))
            self.reg_text = tk.Text(
                rf, height=15, width=30, font=('Monospace', 9),
                state='disabled', bg='#1e1e1e', fg='#d4d4d4', relief='flat')
            self.reg_text.tag_config('chg', foreground='#ff8800')
            self.reg_text.pack()
            bot = ttk.Frame(rf); bot.pack(fill='x', pady=2)
            ttk.Button(bot, text="Refresh", command=self._refresh_regs).pack(side='left', padx=2)
            self._status_labels = {}
            for tag in ('ADL', 'HALT', 'IEF', 'MIX'):
                var = tk.StringVar(value=tag)
                lbl = tk.Label(bot, textvariable=var,
                               font=('Monospace', 8, 'bold'), width=4,
                               relief='solid', bd=1, padx=2,
                               fg='#666666', bg='#2a2a2a')
                lbl.pack(side='left', padx=2)
                self._status_labels[tag] = (var, lbl)

            bf = ttk.LabelFrame(parent, text="Breakpoint", padding=4)
            bf.pack(fill='x', padx=4, pady=2)
            row = ttk.Frame(bf); row.pack(fill='x')
            ttk.Label(row, text="Addr:").pack(side='left')
            self.bp_entry = ttk.Entry(row, width=9)
            self.bp_entry.pack(side='left', padx=4)
            ttk.Button(bf, text="Set & Run to BP",
                       command=self._set_bp_run).pack(pady=2)

            mf = ttk.LabelFrame(parent, text="Memory Dump", padding=4)
            mf.pack(fill='x', padx=4, pady=2)
            row2 = ttk.Frame(mf); row2.pack(fill='x')
            ttk.Label(row2, text="Addr:").pack(side='left')
            self.md_addr = ttk.Entry(row2, width=7)
            self.md_addr.pack(side='left', padx=2)
            ttk.Label(row2, text="Len:").pack(side='left')
            self.md_len = ttk.Entry(row2, width=5)
            self.md_len.insert(0, '80')
            self.md_len.pack(side='left', padx=2)
            btn_row = ttk.Frame(mf); btn_row.pack(pady=2)
            ttk.Button(btn_row, text="Dump", command=self._mem_dump).pack(side='left', padx=2)
            ttk.Button(btn_row, text="Save", command=self._mem_save).pack(side='left', padx=2)

            jf = ttk.LabelFrame(parent, text="Set PC / Run from", padding=4)
            jf.pack(fill='x', padx=4, pady=2)
            row3 = ttk.Frame(jf); row3.pack(fill='x')
            ttk.Label(row3, text="Addr:").pack(side='left')
            self.pc_entry = ttk.Entry(row3, width=9)
            self.pc_entry.pack(side='left', padx=4)
            row4 = ttk.Frame(jf); row4.pack(fill='x', pady=2)
            ttk.Button(row4, text="Set PC",   command=self._set_pc  ).pack(side='left', padx=2)
            ttk.Button(row4, text="Run from", command=self._run_from).pack(side='left', padx=2)

        def _build_right(self, parent):
            ttk.Label(parent, text="Output log").pack(anchor='w', padx=2)
            self.log = scrolledtext.ScrolledText(
                parent, font=('Monospace', 9),
                bg='#1e1e1e', fg='#d4d4d4',
                state='disabled', relief='flat')
            self.log.pack(fill='both', expand=True)

        # helpers

        def _log(self, msg: str):
            self.log.config(state='normal')
            self.log.insert('end', str(msg).rstrip() + '\n')
            self.log.see('end')
            self.log.config(state='disabled')

        def _log_raw(self, text: str):
            self.log.config(state='normal')
            self.log.insert('end', text)
            self.log.see('end')
            self.log.config(state='disabled')

        def _run_async(self, fn, *args):
            """Run fn(*args) in a background thread, streaming stdout live."""
            q = _queue.SimpleQueue()

            class _Stream:
                def write(_, s):
                    if s: q.put(s)
                def flush(_): pass

            def _worker():
                try:
                    with contextlib.redirect_stdout(_Stream()):
                        fn(*args)
                except Exception as e:
                    q.put(f"\nERROR: {e}\n")
                finally:
                    q.put(None)

            threading.Thread(target=_worker, daemon=True).start()
            self._drain_queue(q)

        def _drain_queue(self, q):
            while True:
                try:
                    chunk = q.get_nowait()
                    if chunk is None:
                        return
                    self._log_raw(chunk)
                except _queue.Empty:
                    break
            self.root.after(50, lambda: self._drain_queue(q))

        def _capture(self, fn, *a, **kw):
            buf = io.StringIO()
            try:
                with contextlib.redirect_stdout(buf):
                    result = fn(*a, **kw)
            except Exception as e:
                self._log(f"ERROR: {e}")
                return None
            if buf.getvalue():
                self._log(buf.getvalue())
            return result

        def _need_dbg(self) -> bool:
            if not self.dbg:
                messagebox.showerror("Not connected", "Connect first.")
                return False
            return True

        # toolbar actions

        def _connect(self):
            if self.dbg:
                self._log("Already connected."); return
            try:
                buf = io.StringIO()
                with contextlib.redirect_stdout(buf):
                    self.dbg = ZDIDebugger(self._device)
                    self.dbg.open()
                self._log(buf.getvalue())
                self.status_var.set("Connected")
                was_running = not self.dbg.is_halted()
                if was_running:
                    self.dbg.cpu_halt()
                    self.dbg.wait_halt(timeout=1.0)
                self._refresh_regs()
                if was_running:
                    self.dbg.cpu_run()
            except Exception as e:
                self._log(f"Connect failed: {e}")

        def _disconnect(self):
            if self.dbg:
                self.dbg.close(); self.dbg = None
                self._prev_regs = None
                self.status_var.set("Disconnected")
                self._log("Disconnected.")

        def _reset(self):
            if not self._need_dbg(): return
            buf = io.StringIO()
            try:
                with contextlib.redirect_stdout(buf):
                    self.dbg.cpu_reset()
            except Exception as e:
                if buf.getvalue(): self._log(buf.getvalue().strip())
                self._log(f"ERROR: {e}")
                return
            if buf.getvalue(): self._log(buf.getvalue().strip())
            self._refresh_regs()

        def _halt(self):
            if not self._need_dbg(): return
            self.dbg.cpu_halt()
            self._log("Halted.")
            self._refresh_regs()

        def _run(self):
            if not self._need_dbg(): return
            self.dbg.cpu_run()
            self.status_var.set("Running")
            self._log("Running.")

        def _step(self):
            if not self._need_dbg(): return
            self.dbg.cpu_step()
            if not self.dbg.wait_halt(timeout=2.0):
                self._log("Did not halt after step."); return
            self._log("Stepped.")
            self._refresh_regs()

        def _set_pc(self):
            if not self._need_dbg(): return
            try:
                addr = int(self.pc_entry.get().strip(), 16)
                self.dbg.write_pc(addr)
                self._log(f"PC set to {addr:06X}")
                self._refresh_regs()
            except ValueError:
                self._log("Bad address (hex, no 0x prefix)")

        def _run_from(self):
            if not self._need_dbg(): return
            try:
                addr = int(self.pc_entry.get().strip(), 16)
                self.dbg.write_pc(addr)
                self.dbg.cpu_run()
                self.status_var.set(f"Running from {addr:06X}")
                self._log(f"Running from {addr:06X}")
            except ValueError:
                self._log("Bad address (hex, no 0x prefix)")

        def _refresh_regs(self):
            if not self._need_dbg(): return
            try:
                regs = self.dbg.read_all_regs()
                stat = self.dbg.read_status()
                adl   = bool(stat & STAT_ADL)
                halt  = bool(stat & STAT_HALTED)
                ief   = bool(stat & STAT_IEF)
                mixed = bool(stat & STAT_MIXED)
                prev  = self._prev_regs

                t = self.reg_text
                t.config(state='normal')
                t.delete('1.0', 'end')

                def byt(key):
                    val = regs[key]
                    tag = 'chg' if (prev and prev.get(key) != val) else ''
                    t.insert('end', f'{val:02X}', tag)

                def reg3(name, ku, kh, kl):
                    t.insert('end', f'{name}=')
                    byt(ku); t.insert('end', ' ')
                    byt(kh); t.insert('end', ' ')
                    byt(kl)

                def reg2(name, kh, kl):
                    t.insert('end', f'{name}=')
                    byt(kh); t.insert('end', ' ')
                    byt(kl)

                def flags(fkey):
                    prev_f = prev[fkey] if prev else None
                    f = regs[fkey]
                    t.insert('end', '[')
                    for ch, mask in (('S',0x80),('Z',0x40),('H',0x10),
                                     ('V',0x04),('N',0x02),('C',0x01)):
                        active = bool(f & mask)
                        chg    = prev_f is not None and bool(prev_f & mask) != active
                        t.insert('end', ch if active else ch.lower(),
                                 'chg' if chg else '')
                    t.insert('end', ']')

                mode = 'ADL' if adl else 'Z80'

                # PC  [mode]  MBASE
                t.insert('end', 'PC='); byt('PCU'); byt('PCH'); byt('PCL')
                t.insert('end', f'  [{mode}]  MBASE='); byt('MBASE')
                t.insert('end', '\n')

                # SPS  SPL
                reg2('SPS', 'SPS_H', 'SPS_L')
                t.insert('end', '  ')
                reg3('SPL', 'SPL_U', 'SPL_H', 'SPL_L')
                t.insert('end', '\n')

                # Flags
                t.insert('end', 'Flags: '); flags('F'); t.insert('end', '\n')

                # A, BC, DE, HL, IX, IY – each on its own line
                t.insert('end', 'A='); byt('A'); t.insert('end', '\n')
                reg3('BC', 'BCU', 'B', 'C');     t.insert('end', '\n')
                reg3('DE', 'DEU', 'D', 'E');     t.insert('end', '\n')
                reg3('HL', 'HLU', 'H', 'L');     t.insert('end', '\n')
                reg3('IX', 'IXU', 'IXH', 'IXL'); t.insert('end', '\n')
                reg3('IY', 'IYU', 'IYH', 'IYL'); t.insert('end', '\n')

                # Blank separator
                t.insert('end', '\n')

                # Second bank
                t.insert('end', "Flags': "); flags('pF'); t.insert('end', '\n')
                t.insert('end', "A'="); byt('pA'); t.insert('end', '\n')
                reg3("BC'", 'pBCU', 'pB', 'pC'); t.insert('end', '\n')
                reg3("DE'", 'pDEU', 'pD', 'pE'); t.insert('end', '\n')
                reg3("HL'", 'pHLU', 'pH', 'pL'); t.insert('end', '\n')

                t.config(state='disabled')
                self._prev_regs = dict(regs)

                pc = phys_pc(regs, adl)

                _indicators = [
                    ('ADL',  adl,   'ADL',  'Z80',  '#00cc44', '#1a3a1a', '#ffaa00', '#2a2000'),
                    ('HALT', halt,  'HALT', 'halt', '#ff4444', '#3a1a1a', '#555555', '#2a2a2a'),
                    ('IEF',  ief,   'IEF',  'ief',  '#44aaff', '#1a2a3a', '#555555', '#2a2a2a'),
                    ('MIX',  mixed, 'MIX',  'mix',  '#ffcc00', '#2a2500', '#555555', '#2a2a2a'),
                ]
                for tag, active, on_text, off_text, on_fg, on_bg, off_fg, off_bg in _indicators:
                    var, lbl = self._status_labels[tag]
                    if active:
                        var.set(on_text); lbl.config(fg=on_fg, bg=on_bg)
                    else:
                        var.set(off_text); lbl.config(fg=off_fg, bg=off_bg)

                mem = self.dbg.read_mem(pc, 8)
                self.dbg.write_pc(raw_pc(regs))
                h = ' '.join(f'{b:02X}' for b in mem)
                disasm = ''
                if HAS_DISASM:
                    try:
                        adl    = self.dbg.is_adl()
                        dec    = ez80dis.decode(mem, pc, adl)
                        disasm = f'  =>  {ez80dis.disasm(dec)}'
                    except Exception:
                        pass
                self._log(f"PC={pc:06X}: {h}{disasm}")
                self.status_var.set(f"Halted @ {pc:06X}")
            except Exception as e:
                self._log(f"Register read failed: {e}")

        def _set_bp_run(self):
            if not self._need_dbg(): return
            try:
                addr = int(self.bp_entry.get().strip(), 16)
                self.dbg.set_breakpoint(0, addr)
                self.dbg.cpu_run_to_bp()
                self.status_var.set(f"Running to BP {addr:06X}...")
                self._log(f"Running to breakpoint @ {addr:06X}...")
                self.root.after(200, self._poll_halt)
            except ValueError:
                self._log("Bad breakpoint address (hex, no 0x prefix)")

        def _poll_halt(self):
            if not self.dbg: return
            if self.dbg.is_halted():
                self._log("Breakpoint reached.")
                self._refresh_regs()
            else:
                self.root.after(200, self._poll_halt)

        def _mem_dump(self):
            if not self._need_dbg(): return
            try:
                addr   = int(self.md_addr.get().strip(), 16)
                length = int(self.md_len.get().strip(), 16)
                saved  = self.dbg.read_pc()
                data   = self.dbg.read_mem(addr, length)
                self.dbg.write_pc(saved)
                self._log(f"\nMemory @ {addr:06X}  ({length} bytes):")
                buf = io.StringIO()
                with contextlib.redirect_stdout(buf):
                    hex_dump(data, addr)
                self._log(buf.getvalue())
            except ValueError:
                self._log("Bad address / length")
            except Exception as e:
                self._log(f"Dump failed: {e}")

        def _mem_save(self):
            if not self._need_dbg(): return
            try:
                addr   = int(self.md_addr.get().strip(), 16)
                length = int(self.md_len.get().strip(), 16)
            except ValueError:
                self._log("Bad address / length"); return
            path = filedialog.asksaveasfilename(
                defaultextension='.bin',
                initialfile=f'{addr:06X}_{length:X}.bin',
                filetypes=[('Binary', '*.bin'), ('All', '*.*')])
            if not path: return
            try:
                saved = self.dbg.read_pc()
                data  = self.dbg.read_mem(addr, length)
                self.dbg.write_pc(saved)
                with open(path, 'wb') as f:
                    f.write(bytes(data))
                self._log(f"Saved {length} bytes from {addr:06X} to {path}")
            except Exception as e:
                self._log(f"Save failed: {e}")

        def _flash_dialog(self):
            if not self._need_dbg(): return
            path = filedialog.askopenfilename(
                title="Select Intel HEX file to flash",
                filetypes=[("Intel HEX", "*.hex *.HEX"), ("All files", "*.*")])
            if path:
                self._log(f"Flashing {path}...")
                self._run_async(self.dbg.flash_file, path)

        def _load_dialog(self):
            if not self._need_dbg(): return
            path = filedialog.askopenfilename(
                title="Select Intel HEX file for SRAM",
                filetypes=[("Intel HEX", "*.hex *.HEX"), ("All files", "*.*")])
            if path:
                self._log(f"Loading {path} to SRAM...")
                self._run_async(self.dbg.load_sram, path)


    root = tk.Tk()
    App(root)
    root.mainloop()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

_COMMANDS = {
    'flash':  cmd_flash,
    'load':   cmd_load,
    'run':    cmd_run,
    'break':  cmd_break,
    'dump':   cmd_dump,
    'regs':   cmd_regs,
    'reset':  cmd_reset,
}

def main():
    parser = build_parser()
    args   = parser.parse_args()

    if args.gui:
        launch_gui(args.device)
        return

    try:
        dbg = ZDIDebugger(args.device)
        dbg.open()
    except Exception as e:
        print(f"Connection error: {e}")
        sys.exit(1)

    try:
        _COMMANDS[args.command](dbg, args)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
    finally:
        dbg.close()


if __name__ == '__main__':
    main()
