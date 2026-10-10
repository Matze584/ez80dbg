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
- Unix-socket monitor, protocol-compatible with NGZ80Evo_Tools_Emu --monitor
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
  ez80dbg.py monitor [socket] [--attach]     (default socket: /tmp/ez80dbg.sock)
  ez80dbg.py --gui

Device string (--device DEV)
-----------------------------
  0ABC01B                      serial "0ABC01", port B  (default)
  /2                           first FT4232H found, port B
  ftdi://ftdi:4232h:0ABC01/2   full pyftdi URL (passed through)
"""

import os
import io
import sys
import time
import socket
import select
import atexit
import argparse
import threading
import contextlib

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
        # The GUI and the socket monitor drive the same single ZDI link from
        # two threads.  Every primitive below takes this lock so one MPSSE
        # transaction can never be spliced into another's byte stream; callers
        # hold it across whole sequences (PC save -> read -> PC restore) that
        # must not be observed half-done.  Re-entrant, so nesting is free.
        self.lock  = threading.RLock()

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
        with self.lock:
            self._zdi_start()
            self._codebyte((addr & 0x7F) << 1)
            self._codebyte(value & 0xFF)
            self._ft_write(self._buf)

    def _zdi_write_multiple(self, start_addr: int, data):
        """Write consecutive ZDI registers in one transaction."""
        with self.lock:
            self._zdi_start()
            self._codebyte((start_addr & 0x7F) << 1)
            for b in data:
                self._codebyte(b & 0xFF)
            self._ft_write(self._buf)

    def _zdi_read_reg(self, addr: int) -> int:
        """Read one ZDI register (no extra clock-prep byte)."""
        with self.lock:
            self._zdi_start()
            self._codebyte(((addr & 0x7F) << 1) | 1)
            self._buf += [0x20, 0, 0]                       # MPSSE: read 1 byte
            self._buf += [0x80, RESET_PIN,        ZCL | RESET_PIN]
            self._buf += [0x80, ZCL | RESET_PIN,  ZCL | RESET_PIN]
            self._buf += [0x87]                             # flush USB buffer
            self._ft_write(self._buf)
            return self._ft_read(1)[0]

    def _zdi_read_multiple(self, start_addr: int, count: int) -> list:
        """Read count consecutive ZDI registers in one transaction."""
        with self.lock:
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

    def cpu_run_with_bps(self, enable_mask: int):
        """Run with an explicit ADDR0..3 break-enable mask (0 = run freely)."""
        self._zdi_write_reg(ZDI_BRK_CTL, enable_mask & 0x78)


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
# Unix-socket monitor  (protocol-compatible with NGZ80Evo_Tools_Emu --monitor)
# ---------------------------------------------------------------------------
#
# A plain-text line protocol over a Unix domain socket, single-threaded, that
# speaks exactly the same commands and answer shapes as the emulator's
# monitor.c, so one client can drive either the emulator or real silicon.
#
# Protocol: one command per line in, one or more response lines out, always
# terminated by a line that is either "OK" or "ERR <message>".  An unsolicited
# "BREAK PC=xxxxxx" line can arrive at any time while `continue` is in effect.
#
#   regs                    dump registers
#   mem <addr> [len]        hex+ASCII dump (addr/len in hex, len default 256)
#   disasm <addr> [count]   disassemble `count` instructions (default 10)
#   break <addr>            set a breakpoint (see note on data addresses below)
#   unbreak <addr>          clear a breakpoint
#   breaks                  list breakpoints
#   step [n]                execute n instructions (default 1), synchronously
#   continue | run | c      start free-running; replies immediately
#   pause                   stop free-running
#   reset                   reset + re-init the CPU, halted at PC=0
#   quit | exit             close this connection
#
# ez80dbg-only extensions (the emulator answers "ERR unknown command"):
#
#   load <hexfile>          load an Intel HEX file into SRAM
#   flash <hexfile>         program an Intel HEX file into internal flash
#   setpc <addr>            set PC without running
#   help                    list commands
#
# Differences forced by the hardware, all of them reported honestly:
#   - only four breakpoints (the eZ80F91 has four ZDI address-match registers),
#     where the emulator has sixteen; `break` beyond that answers ERR
#   - those comparators watch the address bus, not just opcode fetches, so a
#     `break` on a data address acts as a watchpoint -- it breaks at the end of
#     the instruction that touched it.  There is no read/write qualifier and no
#     data-value matching; the emulator breaks on PC only.
#   - regs cannot report I, R and IM -- ZDI offers no path to them, so they
#     read as 0.  IEF2 mirrors IEF1 for the same reason.
#   - regs/mem/disasm need a halted CPU; while free-running they answer
#     "ERR cpu is running" instead of a snapshot of a moving target.

MONITOR_DEFAULT_SOCK    = '/tmp/ez80dbg.sock'   # fixed, so clients can hardcode it
MONITOR_MAX_BREAKPOINTS = 4      # eZ80F91 ZDI address-match registers 0..3
MONITOR_BP_ENABLE_BIT   = 0x08   # BRK_CTL bit for ADDR0; ADDR1..3 shift left
MONITOR_STATUS_INTERVAL = 0.02   # seconds between halt polls while running
MONITOR_ACC_LIMIT       = 512    # input line accumulator, as in monitor.c
MONITOR_IDLE_TICK       = 0.2    # idle select() timeout, so stop() is noticed
MONITOR_TRACE_LINES     = 12     # response lines a traced command shows before eliding

NO_LINE = object()   # read_line(): nothing arrived, as opposed to a blank line

# Commands after which the target's visible state may have moved, so a GUI
# watching along has something to redraw.
MONITOR_STATE_CHANGING = frozenset((
    'step', 'continue', 'run', 'c', 'pause', 'reset', 'setpc', 'load', 'flash',
))

MONITOR_HELP = (
    "regs | mem <addr> [len] | disasm <addr> [count]",
    "break <addr> | unbreak <addr> | breaks",
    "step [n] | continue | pause | reset",
    "load <hexfile> | flash <hexfile> | setpc <addr>",
    "quit",
)


class _MonitorError(Exception):
    """Raised for a bad argument; the text goes straight into the ERR line."""


def _mon_num(text: str, base: int) -> int:
    try:
        return int(text, base)
    except ValueError:
        kind = 'hex' if base == 16 else 'decimal'
        raise _MonitorError(f'bad {kind} number: {text}') from None


def _mon_flags_str(f: int) -> str:
    """Flag string in monitor.c's spelling: set letter or '-', S Z H P N C."""
    return ''.join(c if f & m else '-' for c, m in
                   (('S', 0x80), ('Z', 0x40), ('H', 0x10),
                    ('P', 0x04), ('N', 0x02), ('C', 0x01)))


class _MonitorSession:
    """One client connection's worth of state, plus the ZDI plumbing."""

    def __init__(self, dbg: ZDIDebugger, sock_path: str, on_change=None,
                 trace=None):
        self.dbg       = dbg
        self.sock_path = sock_path
        self.on_change = on_change or (lambda: None)
        # trace(command, response_lines) -- command is None for an unsolicited
        # line.  Called from the serving thread, once per completed command, so
        # a GUI gets whole request/response pairs rather than a line drip.
        self.trace     = trace or (lambda cmd, lines: None)
        self._traced   = []
        self.bps       = []       # breakpoint addresses, in the order set
        self.running   = False
        self.conn      = None
        self._acc      = b''

    # ---- wire I/O ---------------------------------------------------------

    def _write(self, text: str):
        if self.conn is None:
            return
        try:
            self.conn.sendall(text.encode('utf-8', 'replace'))
        except OSError:
            pass                  # peer gone; noticed on the next read

    def reply(self, line: str = ''):
        self._traced.append(line)
        self._write(line + '\n')

    def ok(self):
        self._traced.append('OK')
        self._write('OK\n')

    def err(self, msg: str):
        self._traced.append(f'ERR {msg}')
        self._write(f'ERR {msg}\n')

    def read_line(self, timeout):
        """Return a decoded line, NO_LINE on timeout, or None on disconnect.

        timeout is in seconds; None blocks until something arrives.  An empty
        line is a real command (monitor.c answers "ERR empty command"), so it
        must stay distinguishable from having read nothing at all.
        """
        while True:
            nl = self._acc.find(b'\n')
            if nl >= 0:
                line = self._acc[:nl]
                self._acc = self._acc[nl + 1:]
                return line.decode('utf-8', 'replace').rstrip('\r')
            r, _, _ = select.select([self.conn], [], [], timeout)
            if not r:
                return NO_LINE
            try:
                chunk = self.conn.recv(256)
            except OSError:
                return None
            if not chunk:
                return None
            self._acc += chunk
            if len(self._acc) >= MONITOR_ACC_LIMIT:
                self._acc = b''   # line too long: drop it rather than grow

    # ---- target helpers ---------------------------------------------------

    def ensure_halted(self) -> bool:
        self.running = False
        if self.dbg.is_halted():
            return True
        self.dbg.cpu_halt()
        return self.dbg.wait_halt(timeout=2.0)

    def phys_pc_now(self) -> int:
        """Physical PC, MBASE-corrected in Z80 mode. CPU must be halted."""
        adl = self.dbg.is_adl()
        pc  = self.dbg.read_pc()
        if adl:
            return pc
        regs = self.dbg.read_all_regs()
        return phys_pc(regs, False)

    def read_mem_keep_pc(self, addr: int, count: int) -> list:
        """read_mem() drives PC as the address pointer, so save and restore."""
        saved = self.dbg.read_pc()
        try:
            return self.dbg.read_mem(addr, count)
        finally:
            self.dbg.write_pc(saved)

    def arm_breakpoints(self) -> int:
        """Load self.bps into ADDR0.. and return the BRK_CTL enable mask."""
        mask = 0
        for i, addr in enumerate(self.bps[:MONITOR_MAX_BREAKPOINTS]):
            self.dbg.set_breakpoint(i, addr)
            mask |= MONITOR_BP_ENABLE_BIT << i
        return mask

    def step_off_breakpoint(self):
        """Move off an address the break comparators are watching.

        The ZDI_ADDRx comparators watch the address bus, so an opcode fetch at
        a matching address breaks immediately (eZ80F91 PS019217, "ZDI Address
        Match Registers").  Resuming while PC still sits on one therefore
        matches again on the very next fetch and breaks instantly -- the CPU
        never advances and every later breakpoint looks ignored.  BRK_STEP
        writes 0x81, which leaves the four match-enable bits clear, so this
        one instruction escapes the comparators.  monitor.c gets the same
        effect for free by stepping before it tests the new PC.

        A data match needs no such care: it breaks at the *end* of the
        instruction that made the access, so PC is already past the watched
        address and cannot re-trigger.
        """
        if self.bps and self.dbg.is_halted() and self.phys_pc_now() in self.bps:
            self.dbg.cpu_step()
            return self.dbg.wait_halt(HALT_TIMEOUT_DEFAULT)
        return True

    def start_running(self):
        self.step_off_breakpoint()
        self.dbg.cpu_run_with_bps(self.arm_breakpoints())
        self.running = True

    def rearm_if_running(self):
        """Apply a breakpoint edit to a CPU that is already free-running.

        The emulator tests its breakpoint list on every step, so `break` there
        takes effect the moment it is typed; here the comparators have to be
        reloaded to match that.  BRK_CTL is written with brk_next clear, which
        updates the enables without disturbing execution.
        """
        if self.running:
            self.dbg.cpu_run_with_bps(self.arm_breakpoints())

    # ---- command output ---------------------------------------------------

    def dump_regs(self):
        dbg  = self.dbg
        stat = dbg.read_status()
        adl  = bool(stat & STAT_ADL)
        regs = dbg.read_all_regs()
        af  = (regs['A'] << 8) | regs['F']
        bc  = (regs['BCU'] << 16) | (regs['B']   << 8) | regs['C']
        de  = (regs['DEU'] << 16) | (regs['D']   << 8) | regs['E']
        hl  = (regs['HLU'] << 16) | (regs['H']   << 8) | regs['L']
        ix  = (regs['IXU'] << 16) | (regs['IXH'] << 8) | regs['IXL']
        iy  = (regs['IYU'] << 16) | (regs['IYH'] << 8) | regs['IYL']
        sps = regs['SPS_L'] | (regs['SPS_H'] << 8)
        spl = regs['SPL_L'] | (regs['SPL_H'] << 8) | (regs['SPL_U'] << 16)
        pc  = phys_pc(regs, adl)
        ief = 1 if stat & STAT_IEF else 0
        self.reply(f"AF={af:04X} BC={bc:06X} DE={de:06X} HL={hl:06X}")
        self.reply(f"IX={ix:06X} IY={iy:06X} SPS={sps:04X} SPL={spl:06X}")
        # I, R and IM have no ZDI read path on the eZ80F91 -- reported as 0.
        self.reply(f"PC={pc:06X} I=0000 R=00 MBASE={regs['MBASE']:02X}")
        self.reply(
            f"flags={_mon_flags_str(regs['F'])} ADL={1 if adl else 0} "
            f"MADL={1 if stat & STAT_MIXED else 0} IM=0 "
            f"IEF1={ief} IEF2={ief} "
            f"halted={1 if stat & STAT_HALTED else 0}"
        )
        paf = (regs['pA'] << 8) | regs['pF']
        pbc = (regs['pBCU'] << 16) | (regs['pB'] << 8) | regs['pC']
        pde = (regs['pDEU'] << 16) | (regs['pD'] << 8) | regs['pE']
        phl = (regs['pHLU'] << 16) | (regs['pH'] << 8) | regs['pL']
        self.reply(f"AF'={paf:04X} BC'={pbc:06X} DE'={pde:06X} HL'={phl:06X}")

    def dump_mem(self, addr: int, length: int):
        data = self.read_mem_keep_pc(addr, length)
        for off in range(0, length, 16):
            chunk = data[off:off + 16]
            hexpart = ''.join(f'{b:02X} ' for b in chunk)
            hexpart += '   ' * (16 - len(chunk))
            ascii_part = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
            self.reply(f"{(addr + off) & 0xFFFFFF:06X}: {hexpart} {ascii_part}")

    def dump_disasm(self, addr: int, count: int):
        pc  = self.phys_pc_now()
        adl = self.dbg.is_adl()
        # One read covering the whole run; the longest eZ80 opcode is 6 bytes.
        span = min(count * 6 + 6, 0x10000)
        data = self.read_mem_keep_pc(addr, span)
        off = 0
        for _ in range(count):
            here = (addr + off) & 0xFFFFFF
            mnemonic, length = 'db ?', 1
            if HAS_DISASM and off < len(data):
                try:
                    dec      = ez80dis.decode(bytes(data[off:off + 6]), here, adl)
                    mnemonic = ez80dis.disasm(dec)
                    length   = max(1, dec.len)
                except Exception:
                    mnemonic = f'db {data[off]:02X}h'
            self.reply(f"{'>' if here == pc else ' '}"
                       f"{'*' if here in self.bps else ' '}"
                       f"{here:06X}  {mnemonic}")
            off += length

    def forward_output(self, fn, *args):
        """Run fn (which prints progress to stdout) and relay it to the client."""
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            fn(*args)
        for line in buf.getvalue().splitlines():
            if line.strip():
                self.reply(line.rstrip())

    # ---- command dispatch -------------------------------------------------

    def handle(self, line: str) -> bool:
        """Run one command. Returns True if the connection should close."""
        self._traced = []
        try:
            return self._handle(line)
        finally:
            self.trace(line, self._traced)

    def _handle(self, line: str) -> bool:
        parts = line.split()
        if not parts:
            self.err('empty command')
            return False
        cmd, args = parts[0], parts[1:]

        # One command is one transaction against the target: the GUI thread
        # must not slip a PC-clobbering read in between our PC save and
        # restore, so the whole dispatch runs under the ZDI lock.
        try:
            with self.dbg.lock:
                return self._dispatch(cmd, args)
        except _MonitorError as e:
            self.err(str(e))
            return False
        except Exception as e:
            self.err(f'{type(e).__name__}: {e}')
            return False

    def _dispatch(self, cmd: str, args) -> bool:
        needs_halt = cmd in ('regs', 'mem', 'disasm', 'load', 'flash', 'setpc')
        if needs_halt and self.running:
            self.err('cpu is running')
            return False

        if cmd == 'regs':
            self.dump_regs()

        elif cmd == 'mem':
            if not args:
                self.err('usage: mem <addr> [len]')
                return False
            addr   = _mon_num(args[0], 16) & 0xFFFFFF
            length = _mon_num(args[1], 16) if len(args) > 1 else 256
            length = 16 if length <= 0 else min(length, 4096)
            self.dump_mem(addr, length)

        elif cmd == 'disasm':
            if not args:
                self.err('usage: disasm <addr> [count]')
                return False
            addr  = _mon_num(args[0], 16) & 0xFFFFFF
            count = _mon_num(args[1], 10) if len(args) > 1 else 10
            count = max(1, min(count, 256))
            self.dump_disasm(addr, count)

        elif cmd == 'break':
            if not args:
                self.err('usage: break <addr>')
                return False
            addr = _mon_num(args[0], 16) & 0xFFFFFF
            if addr not in self.bps:
                if len(self.bps) >= MONITOR_MAX_BREAKPOINTS:
                    self.err(f'too many breakpoints (hardware limit '
                             f'{MONITOR_MAX_BREAKPOINTS})')
                    return False
                self.bps.append(addr)
            self.rearm_if_running()
            self.reply(f'breakpoint set at {addr:06X}')

        elif cmd == 'unbreak':
            if not args:
                self.err('usage: unbreak <addr>')
                return False
            addr = _mon_num(args[0], 16) & 0xFFFFFF
            if addr in self.bps:
                self.bps.remove(addr)
                self.rearm_if_running()
            self.reply(f'breakpoint cleared at {addr:06X}')

        elif cmd == 'breaks':
            for addr in self.bps:
                self.reply(f'{addr:06X}')

        elif cmd == 'step':
            count = _mon_num(args[0], 10) if args else 1
            count = max(1, min(count, 1000000))
            if not self.ensure_halted():
                self.err('cpu did not halt')
                return False
            for _ in range(count):
                self.dbg.cpu_step()
                if not self.dbg.wait_halt(HALT_TIMEOUT_DEFAULT):
                    self.err('cpu did not halt after step (timeout)')
                    return False
            self.reply(f'PC={self.phys_pc_now():06X}')

        elif cmd in ('continue', 'run', 'c'):
            self.start_running()
            self.reply('running')

        elif cmd == 'pause':
            if not self.ensure_halted():
                self.err('cpu did not halt')
                return False
            self.reply(f'paused PC={self.phys_pc_now():06X}')

        elif cmd == 'reset':
            self.running = False
            with contextlib.redirect_stdout(io.StringIO()):
                self.dbg.cpu_reset()
            self.reply(f'reset PC={self.phys_pc_now():06X}')

        elif cmd == 'load':
            if not args:
                self.err('usage: load <hexfile>')
                return False
            self.forward_output(self.dbg.load_sram, args[0])

        elif cmd == 'flash':
            if not args:
                self.err('usage: flash <hexfile>')
                return False
            self.forward_output(self.dbg.flash_file, args[0])
            self.bps.clear()      # the helper run reprogrammed ADDR0

        elif cmd == 'setpc':
            if not args:
                self.err('usage: setpc <addr>')
                return False
            addr = _mon_num(args[0], 16) & 0xFFFFFF
            if not self.ensure_halted():
                self.err('cpu did not halt')
                return False
            self.dbg.write_pc(addr)
            self.reply(f'PC={self.phys_pc_now():06X}')

        elif cmd == 'help':
            for line in MONITOR_HELP:
                self.reply(line)

        elif cmd in ('quit', 'exit'):
            self.reply('bye')
            self.ok()
            return True

        else:
            self.err('unknown command')
            return False

        if cmd in MONITOR_STATE_CHANGING:
            self.on_change()
        self.ok()
        return False


class MonitorServer:
    """The socket monitor, usable blocking (CLI) or on its own thread (GUI).

    The GUI case is the reason this is a class: the user brings the target
    into a sane state by hand, then hands the same live ZDI link to a socket
    client while the window stays usable.  Both sides serialise on
    ``dbg.lock``; `log`, `on_change` and `trace` are called from the serving
    thread, so a GUI must marshal them back to its own thread itself.
    """

    def __init__(self, dbg: ZDIDebugger, sock_path: str = MONITOR_DEFAULT_SOCK,
                 log=print, on_change=None, trace=None):
        self.dbg       = dbg
        self.sock_path = sock_path
        self.log       = log
        self.session   = _MonitorSession(dbg, sock_path, on_change=on_change,
                                         trace=trace)
        self._srv      = None
        self._thread   = None
        self._stop     = threading.Event()

    # ---- lifecycle --------------------------------------------------------

    @property
    def active(self) -> bool:
        return self._srv is not None

    def bind(self):
        """Create the listening socket. Raises on failure; call before serve."""
        if not hasattr(socket, 'AF_UNIX'):
            raise RuntimeError('Unix domain sockets are not available on this platform')
        if os.path.exists(self.sock_path):
            if self._is_live_socket(self.sock_path):
                raise RuntimeError(f'{self.sock_path} is already served by another process')
            os.unlink(self.sock_path)      # stale file from a crashed run
        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            srv.bind(self.sock_path)
            srv.listen(1)
        except OSError:
            srv.close()
            raise
        self._srv = srv
        self._stop.clear()

    @staticmethod
    def _is_live_socket(path: str) -> bool:
        """True if something is actually accepting on this path right now."""
        probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            probe.settimeout(0.2)
            probe.connect(path)
            return True
        except OSError:
            return False
        finally:
            probe.close()

    def start(self):
        """Bind and serve on a daemon thread; returns once the socket is up."""
        self.bind()
        self._thread = threading.Thread(target=self._serve, name='ez80dbg-monitor',
                                        daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 2.0):
        """Ask the serving thread to finish and clean up the socket file."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
            self._thread = None
        self._close()

    def serve_forever(self):
        """Blocking serve for the CLI; returns on Ctrl-C."""
        try:
            self._serve()
        except KeyboardInterrupt:
            self.log("Monitor stopped.")
        finally:
            self._stop.set()
            self._close()

    def _close(self):
        session = self.session
        if session.conn is not None:
            try:
                session.conn.close()
            except OSError:
                pass
            session.conn = None
        if self._srv is not None:
            self._srv.close()
            self._srv = None
        if os.path.exists(self.sock_path):
            try:
                os.unlink(self.sock_path)
            except OSError:
                pass

    # ---- the loop ---------------------------------------------------------

    def _serve(self):
        session = self.session
        srv     = self._srv
        next_status_check = 0.0

        while not self._stop.is_set():
            # Idle waits are bounded so stop() is noticed promptly; while the
            # target free-runs we do not wait on the socket at all, because the
            # halt poll below has to keep turning.
            wait = 0 if session.running else MONITOR_IDLE_TICK

            if session.conn is None:
                try:
                    r, _, _ = select.select([srv], [], [], wait)
                except (OSError, ValueError):
                    break
                if r:
                    session.conn, _ = srv.accept()
                    session._acc = b''
                    session._write('ez80dbg monitor ready\n')
                    self.log("Monitor: client connected.")

            if session.conn is not None:
                line = session.read_line(wait)
                if line is None:
                    session.conn.close()
                    session.conn = None
                    self.log("Monitor: client disconnected.")
                elif line is not NO_LINE:
                    if session.handle(line):
                        session.conn.close()
                        session.conn = None
                        self.log("Monitor: client disconnected.")

            if session.running:
                now = time.monotonic()
                if now < next_status_check:
                    time.sleep(min(MONITOR_STATUS_INTERVAL, next_status_check - now))
                    continue
                next_status_check = now + MONITOR_STATUS_INTERVAL
                if self.dbg.is_halted():
                    session.running = False
                    session._traced = []
                    session.reply(f'BREAK PC={session.phys_pc_now():06X}')
                    session.trace(None, session._traced)
                    session.on_change()


def monitor_run(dbg: ZDIDebugger, sock_path: str = MONITOR_DEFAULT_SOCK,
                attach: bool = False) -> int:
    """Serve the monitor protocol on a Unix socket until interrupted (CLI)."""
    server  = MonitorServer(dbg, sock_path)
    session = server.session

    if attach:
        state = 'halted' if dbg.is_halted() else 'running'
        session.running = (state == 'running')
        print(f"Attached to a {state} CPU.")
    elif not session.ensure_halted():
        print("Warning: CPU did not halt; attaching anyway.")

    server.bind()
    atexit.register(server._close)
    print(f"Monitor listening on {sock_path} "
          f"({'attached' if attach else 'halted'}; send 'continue' to start)")
    print(f"Connect with:  socat readline UNIX-CONNECT:{sock_path}")
    server.serve_forever()
    return 0


def cmd_monitor(dbg, args):
    """Serve the Unix-socket debug protocol (same commands as the emulator).
    socket: path to create, default /tmp/ez80dbg.sock.
    --attach: leave the CPU alone on start instead of halting it."""
    monitor_run(dbg, args.socket or MONITOR_DEFAULT_SOCK, attach=args.attach)


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

    s = sub.add_parser('monitor', help='Serve the Unix-socket debug protocol')
    s.add_argument('socket', nargs='?', default=None,
                   help=f'Socket path (default: {MONITOR_DEFAULT_SOCK})')
    s.add_argument('--attach', action='store_true',
                   help='Do not halt the CPU on start')



    return p


# ---------------------------------------------------------------------------
# Tkinter GUI
# ---------------------------------------------------------------------------

def launch_gui(device: str):
    import tkinter as tk
    from tkinter import ttk, filedialog, scrolledtext, messagebox
    import io, functools, contextlib, queue as _queue

    def _target_action(what, timeout=5.0):
        """Toolbar action that needs a connection and exclusive ZDI access.

        The socket monitor drives the same link from its own thread, so an
        action arriving mid-command waits briefly and then says so, rather
        than splicing itself into the middle of that command.
        """
        def deco(fn):
            @functools.wraps(fn)
            def wrapper(self, *a, **kw):
                if not self.dbg:
                    messagebox.showerror("Not connected", "Connect first.")
                    return None
                if not self.dbg.lock.acquire(timeout=timeout):
                    self._log(f"Busy: the monitor holds the ZDI link, {what} skipped.")
                    return None
                try:
                    return fn(self, *a, **kw)
                finally:
                    self.dbg.lock.release()
            return wrapper
        return deco

    class App:
        def __init__(self, root: tk.Tk):
            self.root       = root
            self.dbg        = None
            self._device    = device
            self._prev_regs = None
            self.monitor    = None
            self._mon_q     = _queue.SimpleQueue()   # monitor thread -> GUI thread
            root.title("eZ80F91 ZDI Debugger")
            root.minsize(860, 520)
            self._build_ui()
            root.protocol('WM_DELETE_WINDOW', self._on_close)
            self._pump_monitor()

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

            ttk.Separator(tb, orient='vertical').pack(side='left', fill='y', padx=3)
            self.mon_var = tk.BooleanVar(value=False)
            ttk.Checkbutton(tb, text="Monitor", style='Toolbutton', width=10,
                            variable=self.mon_var,
                            command=self._toggle_monitor).pack(
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

        def _locked(self, fn):
            """Wrap a worker-thread callable so it owns the ZDI link while it runs."""
            def run(*a, **kw):
                with self.dbg.lock:
                    return fn(*a, **kw)
            return run

        # monitor

        def _toggle_monitor(self):
            if self.mon_var.get():
                self._monitor_start()
            else:
                self._monitor_stop()

        def _monitor_start(self):
            if not self.dbg:
                messagebox.showerror("Not connected", "Connect first.")
                self.mon_var.set(False)
                return
            if self.monitor is not None:
                return
            server = MonitorServer(self.dbg, MONITOR_DEFAULT_SOCK,
                                   log=self._mon_log, on_change=self._mon_changed,
                                   trace=self._mon_trace)
            # Take the target exactly as the user left it -- getting it into a
            # sane state by hand first is the whole point of this button.
            server.session.running = not self.dbg.is_halted()
            try:
                server.start()
            except Exception as e:
                self._log(f"Monitor failed to start: {e}")
                self.mon_var.set(False)
                return
            self.monitor = server
            self._log(f"Monitor listening on {MONITOR_DEFAULT_SOCK}")
            self._log(f"  socat readline UNIX-CONNECT:{MONITOR_DEFAULT_SOCK}")
            self.status_var.set(f"Monitor on {MONITOR_DEFAULT_SOCK}")

        def _monitor_stop(self):
            if self.monitor is not None:
                self.monitor.stop()
                self.monitor = None
                self._log("Monitor stopped.")
                self.status_var.set("Monitor off")
            self.mon_var.set(False)

        def _mon_log(self, msg):
            self._mon_q.put(('log', str(msg)))       # called from the monitor thread

        def _mon_changed(self):
            self._mon_q.put(('refresh', None))       # called from the monitor thread

        def _mon_trace(self, cmd, lines):
            """One socket command and its answer, for the log (monitor thread).

            Long dumps are elided here rather than in the log so a `mem` of a
            few KB cannot bury the rest of the window.
            """
            shown = list(lines[:MONITOR_TRACE_LINES])
            if len(lines) > MONITOR_TRACE_LINES:
                shown.append(f'... ({len(lines) - MONITOR_TRACE_LINES} more lines)')
            self._mon_q.put(('trace', (cmd, shown)))

        def _pump_monitor(self):
            """Drain the monitor thread's messages; Tk is touched only here."""
            refresh = False
            while True:
                try:
                    kind, payload = self._mon_q.get_nowait()
                except _queue.Empty:
                    break
                if kind == 'log':
                    self._log(payload)
                elif kind == 'trace':
                    cmd, lines = payload
                    # 'socket>' what the client asked, 'socket<' a line it got
                    # without asking (a BREAK), indented for everything else.
                    if cmd is None:
                        for line in lines:
                            self._log(f'socket< {line}')
                    else:
                        self._log(f'socket> {cmd}')
                        for line in lines:
                            self._log(f'        {line}')
                else:
                    refresh = True                   # coalesce a burst into one
            # Skip the auto-refresh while the monitor is mid-command (a flash
            # holds the link for minutes); its next state change queues another.
            if refresh and self.dbg and self.dbg.lock.acquire(blocking=False):
                self.dbg.lock.release()
                self._refresh_regs()
            self.root.after(200, self._pump_monitor)

        def _on_close(self):
            self._monitor_stop()
            if self.dbg:
                self.dbg.close()
            self.root.destroy()

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
            self._monitor_stop()
            if self.dbg:
                self.dbg.close(); self.dbg = None
                self._prev_regs = None
                self.status_var.set("Disconnected")
                self._log("Disconnected.")

        @_target_action('reset')
        def _reset(self):
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

        @_target_action('halt')
        def _halt(self):
            self.dbg.cpu_halt()
            self._log("Halted.")
            self._refresh_regs()

        @_target_action('run')
        def _run(self):
            self.dbg.cpu_run()
            self.status_var.set("Running")
            self._log("Running.")

        @_target_action('step')
        def _step(self):
            self.dbg.cpu_step()
            if not self.dbg.wait_halt(timeout=2.0):
                self._log("Did not halt after step."); return
            self._log("Stepped.")
            self._refresh_regs()

        @_target_action('set PC')
        def _set_pc(self):
            try:
                addr = int(self.pc_entry.get().strip(), 16)
                self.dbg.write_pc(addr)
                self._log(f"PC set to {addr:06X}")
                self._refresh_regs()
            except ValueError:
                self._log("Bad address (hex, no 0x prefix)")

        @_target_action('run-from')
        def _run_from(self):
            try:
                addr = int(self.pc_entry.get().strip(), 16)
                self.dbg.write_pc(addr)
                self.dbg.cpu_run()
                self.status_var.set(f"Running from {addr:06X}")
                self._log(f"Running from {addr:06X}")
            except ValueError:
                self._log("Bad address (hex, no 0x prefix)")

        @_target_action('refresh')
        def _refresh_regs(self):
            try:
                stat = self.dbg.read_status()
                if not stat & STAT_ZDI:
                    # CPU is free-running: reading now would clobber PC and
                    # report noise.  Halt (or let a breakpoint hit) first.
                    self.status_var.set("Running - halt to read registers")
                    return
                regs = self.dbg.read_all_regs()
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

        @_target_action('breakpoint run')
        def _set_bp_run(self):
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

        @_target_action('dump')
        def _mem_dump(self):
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
                with self.dbg.lock:
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
                self._run_async(self._locked(self.dbg.flash_file), path)

        def _load_dialog(self):
            if not self._need_dbg(): return
            path = filedialog.askopenfilename(
                title="Select Intel HEX file for SRAM",
                filetypes=[("Intel HEX", "*.hex *.HEX"), ("All files", "*.*")])
            if path:
                self._log(f"Loading {path} to SRAM...")
                self._run_async(self._locked(self.dbg.load_sram), path)


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
    'monitor': cmd_monitor,
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
