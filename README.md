# ez80dbg — eZ80F91 ZDI Flash Loader and Debugger

![GUI screenshot](docs/screenshot.png)

A Python tool for flashing, loading, and debugging firmware on the **eZ80F91**
microcontroller via the **Zilog Debug Interface (ZDI)**.  Communication runs
over an FTDI FT4232H USB adapter using pyftdi.

---

## Features

| Feature | Description |
|---|---|
| **Flash / Load** | Write Intel HEX files to internal flash or SRAM |
| **Run / Halt / Step** | Full CPU control — run, halt, reset, single-step |
| **Breakpoints** | Set a hardware breakpoint, run, wait for halt, then step |
| **Registers** | Full register dump with inline eZ80/Z80 disassembly at PC |
| **Memory dump** | Hex dump any address range |
| **GUI** | Tkinter interface with register panel, disassembly and live log |
| **ADL-aware disassembler** | Standalone `ez80dis.py` — decodes eZ80 prefixes (`.lil/.lis/.sil/.sis`), new eZ80 instructions, and 24-bit operands in ADL mode |

---

## Hardware requirements

| Component | Details |
|---|---|
| Adapter | FTDI FT4232H (Port B, MPSSE) |
| Target | eZ80F91 with ZDI pins exposed |
| Wiring | ZCL → Port B bit 0 · ZDA → Port B bit 1 and 2 wired together · needs about 10k PullUp to +3V3|

The default device string assumes serial number `0ABC01`, Port B.  
Override with `--device` if your adapter differs (see below).

---

## Installation

### Linux / macOS

```bash
git clone <repo-url>
cd ez80dbg
./setup.sh
```

**Linux — USB access without root:**

```bash
# udev rule for FT4232H
echo 'ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6011", MODE="0660", GROUP="plugdev"' \
  | sudo tee /etc/udev/rules.d/99-ftdi.rules
sudo udevadm control --reload-rules
sudo usermod -aG plugdev $USER   # log out and back in
```

### Windows

```
setup.bat
```

pyftdi uses **libusb**, not the default FTDI D2XX driver.  
Replace the driver for FT4232H Interface B using [Zadig](https://zadig.akeo.ie/):
`Options → List All Devices → select Interface B → WinUSB → Install Driver`

---

## Usage

### CLI

```
usage: ez80dbg [-h] [--device DEV] [--gui]
               {flash,load,run,break,dump,regs,reset} ...
```

| Command | Synopsis | Description |
|---|---|---|
| `flash` | `<hexfile> [--run ADDR]` | Write HEX file to internal flash |
| `load` | `<hexfile> [--run [ADDR]] [--step]` | Load HEX file into SRAM |
| `run` | `<addr> [--step]` | Start execution at address |
| `break` | `<addr> [--reset] [--timeout SEC]` | Set breakpoint, run, wait, step |
| `dump` | `<addr> [<length>]` | Hex dump a memory region |
| `regs` | | Show CPU registers + disassembly at PC |
| `reset` | `[--run ADDR]` | Reset and halt CPU |

**Tip:** Instead of prefixing every command with `venv/bin/python3`, activate
the venv once for the session:

```bash
source venv/bin/activate        # Linux / macOS
venv\Scripts\activate.bat       # Windows

python3 ez80dbg.py --help       # no venv prefix needed until shell is closed
```

**Examples:**

```bash
# Flash and run from firmware entry point
venv/bin/python3 ez80dbg.py flash firmware.hex --run 0x000000

# Load into SRAM and single-step from the start
venv/bin/python3 ez80dbg.py load firmware.hex --run --step

# Set breakpoint at 0xC000, run, wait up to 10 s
venv/bin/python3 ez80dbg.py break 0xC000 --timeout 10

# Hex dump 256 bytes at 0x40000
venv/bin/python3 ez80dbg.py dump 0x40000 256

# Use a specific FTDI adapter
venv/bin/python3 ez80dbg.py --device 0DEF02B regs
```

### Device string format

```
0ABC01B                      serial "0ABC01", port B  (default)
0ABC01A                      serial "0ABC01", port A
/2                           first FT4232H found, port B
ftdi://ftdi:4232h:0ABC01/2   full pyftdi URL
```

### GUI

```bash
venv/bin/python3 ez80dbg.py --gui
```

The GUI provides the same functionality via toolbar buttons.
Connect/Disconnect manages the FTDI link; all other buttons require an active
connection.

---

## Dependencies

| Package | Purpose |
|---|---|
| [pyftdi](https://github.com/eblot/pyftdi) | FTDI USB adapter access via libusb |
| [intelhex](https://github.com/python-intelhex/intelhex) | Intel HEX file parsing |
| [ez80dis](https://github.com/Matze584/ez80dis) | eZ80 Disassembler |
| tkinter | GUI Framework |

Install via `setup.sh` / `setup.bat`, or manually:

```bash
python3 -m venv venv
venv/bin/pip install pyftdi intelhex
```

---

## License

see [SPDX identifier](ez80dbg.py) in source header.

The Disassembler is based on z80dis (https://github.com/lwerdna/z80dis)
It is licensed under "The Unlicense". ez80dis is also. 

