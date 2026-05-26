#!/usr/bin/env python3
"""ADL-aware Z80/eZ80 disassembler.

Based on z80dis (https://github.com/lwerdna/z80dis)

API:
    dec = decode(data, addr=0, adl=False)  -> Decoded
    txt = disasm(dec_or_data, pc=0, adl=False) -> str

# SPDX-License-Identifier: The Unlicense
"""

__version__ = "1.0.1"

# eZ80-specific instructions:
#
# Mode-prefix bytes (override ADL mode for the following instruction):
#   .SIS (40)   .LIS (49)   .SIL (52)   .LIL (5B)
#
# ED — IN0 / OUT0 / TST / TSTIO:
#   IN0 r,(n)     (ED 00/08/10/18/20/28/38 n)       r = B/C/D/E/H/L/A  (ED 30 = NONI)
#   OUT0 (n),r    (ED 01/09/11/19/21/29/39 n)       r = B/C/D/E/H/L/A
#   TST A,r       (ED 04/0C/14/1C/24/2C/34/3C)      r = B/C/D/E/H/L/(HL)/A
#   TST A,n       (ED 64 n)   TSTIO n   (ED 74 n)
#
# ED — LEA / PEA:
#   LEA BC,IX+d  (ED 02 d)   LEA BC,IY+d  (ED 03 d)
#   LEA DE,IX+d  (ED 12 d)   LEA DE,IY+d  (ED 13 d)
#   LEA HL,IX+d  (ED 22 d)   LEA HL,IY+d  (ED 23 d)
#   LEA IX,IX+d  (ED 32 d)   LEA IY,IY+d  (ED 33 d)
#   LEA IX,IY+d  (ED 54 d)   LEA IY,IX+d  (ED 55 d)
#   PEA IX+d     (ED 65 d)   PEA IY+d     (ED 66 d)
#
# ED — MLT / SLP / STMIX / RSMIX / LD MB:
#   MLT BC  (ED 4C)   MLT DE  (ED 5C)   MLT HL  (ED 6C)   MLT SP  (ED 7C)
#   LD MB,A (ED 6D)   LD A,MB (ED 6E)
#   SLP     (ED 76)   STMIX   (ED 7D)   RSMIX   (ED 7E)
#
# ED — 16-bit LD:
#   LD BC,(HL)  (ED 07)   LD DE,(HL)  (ED 17)   LD HL,(HL)  (ED 27)
#   LD IX,(HL)  (ED 37)   LD IY,(HL)  (ED 31)
#   LD (HL),BC  (ED 0F)   LD (HL),DE  (ED 1F)   LD (HL),HL  (ED 2F)
#   LD (HL),IX  (ED 3F)   LD (HL),IY  (ED 3E)
#   LD I,HL     (ED C7)   LD HL,I     (ED D7)
#
# ED — Extended block I/O:
#   INIRX  (ED C2)   INDRX  (ED CA)   OTIRX  (ED C3)   OTDRX  (ED CB)
#   INIM   (ED 82)   INDM   (ED 8A)   INIMR  (ED 92)   INDMR  (ED 9A)
#   OTIM   (ED 83)   OTDM   (ED 8B)   OTIMR  (ED 93)   OTDMR  (ED 9B)
#   INI2   (ED 84)   IND2   (ED 8C)   INI2R  (ED 94)   IND2R  (ED 9C)
#   OUTI2  (ED A4)   OUTD2  (ED AC)   OTI2R  (ED B4)   OTD2R  (ED BC)
#
# DD/FD — 16-bit LD with displacement:
#   LD BC,(IX+d)  (DD 07 d)   LD BC,(IY+d)  (FD 07 d)
#   LD DE,(IX+d)  (DD 17 d)   LD DE,(IY+d)  (FD 17 d)
#   LD HL,(IX+d)  (DD 27 d)   LD HL,(IY+d)  (FD 27 d)
#   LD IY,(IX+d)  (DD 31 d)   LD IX,(IY+d)  (FD 31 d)
#   LD IX,(IX+d)  (DD 37 d)   LD IY,(IY+d)  (FD 37 d)
#   LD (IX+d),BC  (DD 0F d)   LD (IY+d),BC  (FD 0F d)
#   LD (IX+d),DE  (DD 1F d)   LD (IY+d),DE  (FD 1F d)
#   LD (IX+d),HL  (DD 2F d)   LD (IY+d),HL  (FD 2F d)
#   LD (IX+d),IY  (DD 3E d)   LD (IY+d),IX  (FD 3E d)
#   LD (IX+d),IX  (DD 3F d)   LD (IY+d),IY  (FD 3F d)

from enum import Enum, auto, unique


# ---------------------------------------------------------------------------
# Enumerations
# ---------------------------------------------------------------------------

@unique
class DECODE_STATUS(Enum):
    OK = 0
    INVALID_INSTRUCTION = auto()
    ERROR = auto()

@unique
class INSTRTYPE(Enum):
    NOP = 0
    LOAD_EXCHANGE = auto()
    BLOCK_TRANSFER_SEARCH = auto()
    ARITHMETIC_LOGICAL = auto()
    ROTATE_SHIFT = auto()
    BIT_MANIPULATION = auto()
    JUMP_CALL_RETURN = auto()
    INPUT_OUTPUT = auto()
    CPU_CONTROL = auto()

@unique
class OPER_TYPE(Enum):
    NONE = 0
    REG = auto()
    REG_DEREF = auto()
    ADDR = auto()
    ADDR_DEREF = auto()
    MEM_DISPL_IX = auto()   # (IX+d) — memory dereference
    MEM_DISPL_IY = auto()   # (IY+d)
    DISPL_IX = auto()       # IX+d  — effective address, no dereference (LEA/PEA)
    DISPL_IY = auto()       # IY+d
    PORT_DEREF = auto()     # (n)   — 8-bit immediate port (IN0/OUT0)
    IMM = auto()
    COND = auto()

@unique
class REG(Enum):
    NONE = 0
    A = auto(); F = auto()
    B = auto(); C = auto()
    D = auto(); E = auto()
    H = auto(); L = auto()
    AF = auto(); BC = auto(); DE = auto(); HL = auto()
    A_ = auto(); F_ = auto()
    B_ = auto(); C_ = auto()
    D_ = auto(); E_ = auto()
    H_ = auto(); L_ = auto()
    AF_ = auto(); BC_ = auto(); DE_ = auto(); HL_ = auto()
    I = auto(); R = auto()
    IX = auto(); IXH = auto(); IXL = auto()
    IY = auto(); IYH = auto(); IYL = auto()
    SP = auto(); PC = auto(); MB = auto()

@unique
class OP(Enum):
    NOP = 0
    NONI = auto()
    # LOAD_EXCHANGE
    LD = auto()
    EX = auto(); EXX = auto()
    POP = auto(); PUSH = auto()
    LEA = auto()    # eZ80: Load Effective Address
    PEA = auto()    # eZ80: Push Effective Address
    MARKER_END_LOAD_EXCHANGE = auto()
    # BLOCK_TRANSFER_SEARCH
    LDI = auto(); CPI = auto(); INI = auto(); OUTI = auto()
    LDD = auto(); CPD = auto(); IND = auto(); OUTD = auto()
    LDIR = auto(); CPIR = auto(); INIR = auto(); OTIR = auto()
    LDDR = auto(); CPDR = auto(); INDR = auto(); OTDR = auto()
    INIRX = auto(); OTIRX = auto(); OTDRX = auto()                   # eZ80
    INDRX = auto()                                                   # eZ80
    INIM = auto(); INIMR = auto()                                    # eZ80
    INDM = auto(); INDMR = auto()                                    # eZ80
    INI2 = auto(); INI2R = auto()                                    # eZ80
    IND2 = auto(); IND2R = auto()                                    # eZ80
    OUTI2 = auto(); OUTD2 = auto()                                   # eZ80
    OTI2R = auto(); OTD2R = auto()                                   # eZ80
    OTIM = auto(); OTIMR = auto()                                    # eZ80
    OTDM = auto(); OTDMR = auto()                                    # eZ80
    MARKER_END_BLOCK_TRANSFER_SEARCH = auto()
    # ARITHMETIC_LOGICAL
    ADD = auto(); ADC = auto(); SUB = auto(); SBC = auto()
    AND = auto(); XOR = auto(); OR = auto(); CP = auto()
    RLC = auto(); RRC = auto(); RL = auto(); RR = auto()
    SLA = auto(); SRA = auto(); SLL = auto(); SRL = auto()
    INC = auto(); DEC = auto(); NEG = auto()
    DAA = auto(); CPL = auto(); SCF = auto(); CCF = auto()
    TST = auto()    # eZ80: Test (AND without storing)
    MLT = auto()    # eZ80: Multiply
    MARKER_END_ARITHMETIC_LOGICAL = auto()
    # ROTATE_SHIFT
    RLCA = auto(); RRCA = auto(); RLA = auto(); RRA = auto()
    RRD = auto(); RLD = auto()
    MARKER_END_ROTATE_SHIFT = auto()
    # BIT_MANIPULATION
    BIT = auto(); RES = auto(); SET = auto()
    MARKER_END_BIT_MANIPULATION = auto()
    # JUMP_CALL_RETURN
    CALL = auto(); RET = auto()
    JP = auto(); JR = auto(); DJNZ = auto()
    RETI = auto(); RETN = auto()
    MARKER_END_JUMP_CALL_RETURN = auto()
    # INPUT_OUTPUT
    IN = auto(); OUT = auto()
    IN0 = auto(); OUT0 = auto()     # eZ80: IN/OUT with immediate port
    TSTIO = auto()                  # eZ80: Test I/O port
    MARKER_END_INPUT_OUTPUT = auto()
    # CPU_CONTROL
    HALT = auto(); DI = auto(); EI = auto(); IM = auto(); RST = auto()
    STMIX = auto()  # eZ80: Set Mixed Memory Mode
    RSMIX = auto()  # eZ80: Reset Mixed Memory Mode
    SLP = auto()    # eZ80: Sleep Mode
    MARKER_END_CPU_CONTROL = auto()

@unique
class CC(Enum):
    ALWAYS = 0
    C = auto();   NOT_C = auto()
    N = auto();   NOT_N = auto()
    P = auto();   NOT_P = auto()
    H = auto();   NOT_H = auto()
    Z = auto();   NOT_Z = auto()
    S = auto();   NOT_S = auto()

@unique
class PREFIX(Enum):
    NONE = 0
    CB = auto(); ED = auto()
    DD = auto(); FD = auto()
    DDCB = auto(); FDCB = auto()


# ---------------------------------------------------------------------------
# Decoded result
# ---------------------------------------------------------------------------

class Decoded:
    def __init__(self):
        self.status      = DECODE_STATUS.ERROR
        self.len         = 0
        self.typ         = None
        self.op          = None
        self.operands    = []
        self.metaLoad    = None
        self.mode_suffix = ''       # '.lil' / '.lis' / '.sil' / '.sis' / ''
        self.long_op     = False    # True → 24-bit immediates/addresses active


# ---------------------------------------------------------------------------
# Decode tables
# ---------------------------------------------------------------------------

TABLE_R = [
    (OPER_TYPE.REG,       REG.B),
    (OPER_TYPE.REG,       REG.C),
    (OPER_TYPE.REG,       REG.D),
    (OPER_TYPE.REG,       REG.E),
    (OPER_TYPE.REG,       REG.H),
    (OPER_TYPE.REG,       REG.L),
    (OPER_TYPE.REG_DEREF, REG.HL),
    (OPER_TYPE.REG,       REG.A),
]

TABLE_RP = [
    (OPER_TYPE.REG, REG.BC),
    (OPER_TYPE.REG, REG.DE),
    (OPER_TYPE.REG, REG.HL),
    (OPER_TYPE.REG, REG.SP),
]

TABLE_RP2 = [
    (OPER_TYPE.REG, REG.BC),
    (OPER_TYPE.REG, REG.DE),
    (OPER_TYPE.REG, REG.HL),
    (OPER_TYPE.REG, REG.AF),
]

# eZ80 LEA destination pairs: p=3 is IX (z=2) or IY (z=3), not SP
TABLE_LEA_IX = [
    (OPER_TYPE.REG, REG.BC),
    (OPER_TYPE.REG, REG.DE),
    (OPER_TYPE.REG, REG.HL),
    (OPER_TYPE.REG, REG.IX),
]
TABLE_LEA_IY = [
    (OPER_TYPE.REG, REG.BC),
    (OPER_TYPE.REG, REG.DE),
    (OPER_TYPE.REG, REG.HL),
    (OPER_TYPE.REG, REG.IY),
]

TABLE_CC = [
    CC.NOT_Z, CC.Z,
    CC.NOT_C, CC.C,
    CC.NOT_P, CC.P,
    CC.NOT_S, CC.S,
]

TABLE_ALU_OP = [
    OP.ADD, OP.ADC, OP.SUB, OP.SBC,
    OP.AND, OP.XOR, OP.OR,  OP.CP,
]

TABLE_ROT = [
    OP.RLC, OP.RRC, OP.RL, OP.RR,
    OP.SLA, OP.SRA, OP.SLL, OP.SRL,
]

TABLE_IM = [0, 0, 1, 2, 0, 0, 1, 2]

TABLE_BLI = [
    [OP.LDI,  OP.CPI,  OP.INI,  OP.OUTI],
    [OP.LDD,  OP.CPD,  OP.IND,  OP.OUTD],
    [OP.LDIR, OP.CPIR, OP.INIR, OP.OTIR],
    [OP.LDDR, OP.CPDR, OP.INDR, OP.OTDR],
]

TABLE_ASSORTED = [
    OP.RLCA, OP.RRCA, OP.RLA, OP.RRA,
    OP.DAA,  OP.CPL,  OP.SCF, OP.CCF,
]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def int8(b):
    return -((b ^ 0xFF) + 1) if b & 0x80 else b

def uint16(b0, b1):
    return b0 | (b1 << 8)

def uint24(b0, b1, b2):
    return b0 | (b1 << 8) | (b2 << 16)

def load_imm(data, offset, long_op):
    """Return (value, byte_count) for a 16- or 24-bit little-endian immediate."""
    if long_op:
        return uint24(data[offset], data[offset+1], data[offset+2]), 3
    return uint16(data[offset], data[offset+1]), 2

def reorder(dc):
    if len(dc.operands) == 2:
        dc.operands[0], dc.operands[1] = dc.operands[1], dc.operands[0]

def xyz(byte):
    x = byte >> 6
    y = (byte >> 3) & 7
    z = byte & 7
    p = y >> 1
    q = y & 1
    return x, y, z, p, q

def will_deref_hl(opc):
    x, y, z, p, q = xyz(opc)
    if x == 0 and z in (4, 5, 6) and y == 6:
        return True
    if x == 1 and not (z == 6 and y == 6) and (y == 6 or z == 6):
        return True
    if x == 2 and z == 6:
        return True
    if x == 3 and z == 1 and q == 1 and p == 2:
        return True
    return False


# ---------------------------------------------------------------------------
# Decode functions
# ---------------------------------------------------------------------------

def decode_unprefixed(data, addr, result, long_op):
    x, y, z, p, q = xyz(data[0])
    result.len += 1

    if x == 0:
        if z == 0:
            if y == 0:
                result.op = OP.NOP
            elif y == 1:
                result.op = OP.EX
                result.operands += [(OPER_TYPE.REG, REG.AF), (OPER_TYPE.REG, REG.AF_)]
            elif y == 2:
                result.op = OP.DJNZ
                result.operands.append((OPER_TYPE.ADDR, addr + 2 + int8(data[1])))
                result.len += 1
            else:   # y = 3..7: JR [cc,] e
                result.op = OP.JR
                result.operands.append((OPER_TYPE.COND, TABLE_CC[y - 4]))
                result.operands.append((OPER_TYPE.ADDR, addr + 2 + int8(data[1])))
                result.len += 1
                if y == 3:
                    result.operands = [result.operands[1]]   # unconditional JR

        elif z == 1:
            if q == 0:  # LD rp, nn  (16 or 24-bit)
                imm, nb = load_imm(data, 1, long_op)
                result.op = OP.LD
                result.operands.append(TABLE_RP[p])
                result.operands.append((OPER_TYPE.IMM, imm))
                result.len += nb
            else:       # ADD HL, rp
                result.op = OP.ADD
                result.operands += [(OPER_TYPE.REG, REG.HL), TABLE_RP[p]]

        elif z == 2:    # LD (BC/DE/nn), A  and reverses
            result.op = OP.LD
            if p == 0:
                result.operands += [(OPER_TYPE.REG_DEREF, REG.BC), (OPER_TYPE.REG, REG.A)]
            elif p == 1:
                result.operands += [(OPER_TYPE.REG_DEREF, REG.DE), (OPER_TYPE.REG, REG.A)]
            else:
                addr_val, nb = load_imm(data, 1, long_op)
                result.len += nb
                result.operands.append((OPER_TYPE.ADDR_DEREF, addr_val))
                result.operands.append((OPER_TYPE.REG, REG.HL) if p == 2 else (OPER_TYPE.REG, REG.A))
            if q:
                reorder(result)

        elif z == 3:
            result.op = OP.DEC if q else OP.INC
            result.operands.append(TABLE_RP[p])

        elif z == 4:
            result.op = OP.INC
            result.operands.append(TABLE_R[y])

        elif z == 5:
            result.op = OP.DEC
            result.operands.append(TABLE_R[y])

        elif z == 6:    # LD r, n
            result.op = OP.LD
            result.operands += [TABLE_R[y], (OPER_TYPE.IMM, data[1])]
            result.len += 1

        elif z == 7:
            result.op = TABLE_ASSORTED[y]

    elif x == 1:
        if z == 6 and y == 6:
            result.op = OP.HALT
        else:
            result.op = OP.LD
            result.operands += [TABLE_R[y], TABLE_R[z]]

    elif x == 2:
        result.op = TABLE_ALU_OP[y]
        result.operands += [(OPER_TYPE.REG, REG.A), TABLE_R[z]]
        if result.op in (OP.SUB, OP.AND, OP.XOR, OP.OR, OP.CP):
            result.operands = [result.operands[1]]

    elif x == 3:
        if z == 0:      # RET cc
            result.op = OP.RET
            result.operands.append((OPER_TYPE.COND, TABLE_CC[y]))

        elif z == 1:
            if q:
                if p == 0: result.op = OP.RET
                elif p == 1: result.op = OP.EXX
                elif p == 2:
                    result.op = OP.JP
                    result.operands.append((OPER_TYPE.REG_DEREF, REG.HL))
                elif p == 3:
                    result.op = OP.LD
                    result.operands += [(OPER_TYPE.REG, REG.SP), (OPER_TYPE.REG, REG.HL)]
            else:
                result.op = OP.POP
                result.operands.append(TABLE_RP2[p])

        elif z == 2:    # JP cc, nn
            addr_val, nb = load_imm(data, 1, long_op)
            result.op = OP.JP
            result.operands += [(OPER_TYPE.COND, TABLE_CC[y]), (OPER_TYPE.ADDR, addr_val)]
            result.len += nb

        elif z == 3:
            if y == 0:      # JP nn
                addr_val, nb = load_imm(data, 1, long_op)
                result.op = OP.JP
                result.operands.append((OPER_TYPE.ADDR, addr_val))
                result.len += nb
            elif y == 1:
                pass        # CB prefix — handled in decode()
            elif y == 2:    # OUT (n), A
                result.op = OP.OUT
                result.operands += [(OPER_TYPE.PORT_DEREF, data[1]), (OPER_TYPE.REG, REG.A)]
                result.len += 1
            elif y == 3:    # IN A, (n)
                result.op = OP.IN
                result.operands += [(OPER_TYPE.REG, REG.A), (OPER_TYPE.PORT_DEREF, data[1])]
                result.len += 1
            elif y == 4:    # EX (SP), HL
                result.op = OP.EX
                result.operands += [(OPER_TYPE.REG_DEREF, REG.SP), (OPER_TYPE.REG, REG.HL)]
            elif y == 5:    # EX DE, HL
                result.op = OP.EX
                result.operands += [(OPER_TYPE.REG, REG.DE), (OPER_TYPE.REG, REG.HL)]
            elif y == 6:
                result.op = OP.DI
            elif y == 7:
                result.op = OP.EI

        elif z == 4:    # CALL cc, nn
            addr_val, nb = load_imm(data, 1, long_op)
            result.op = OP.CALL
            result.operands += [(OPER_TYPE.COND, TABLE_CC[y]), (OPER_TYPE.ADDR, addr_val)]
            result.len += nb

        elif z == 5:
            if q:
                if not p:   # CALL nn
                    addr_val, nb = load_imm(data, 1, long_op)
                    result.op = OP.CALL
                    result.operands.append((OPER_TYPE.ADDR, addr_val))
                    result.len += nb
                # else: DD/ED/FD prefix byte — handled by decode()
            else:           # PUSH rp2
                result.op = OP.PUSH
                result.operands.append(TABLE_RP2[p])

        elif z == 6:    # alu A, n
            result.op = TABLE_ALU_OP[y]
            result.operands += [(OPER_TYPE.REG, REG.A), (OPER_TYPE.IMM, data[1])]
            result.len += 1
            if result.op in (OP.SUB, OP.AND, OP.XOR, OP.OR, OP.CP):
                result.operands = [result.operands[1]]

        elif z == 7:    # RST y*8
            result.op = OP.RST
            result.operands.append((OPER_TYPE.IMM, 8 * y))


def decode_cb(data, addr, result):
    x, y, z, p, q = xyz(data[0])
    result.len += 1
    if x:
        result.op = (None, OP.BIT, OP.RES, OP.SET)[x]
        result.operands += [(OPER_TYPE.IMM, y), TABLE_R[z]]
    else:
        result.op = TABLE_ROT[y]
        result.operands.append(TABLE_R[z])


def decode_ed(data, addr, result, long_op):
    x, y, z, p, q = xyz(data[0])
    result.len += 1

    if x == 0:
        # ── eZ80 additions (all NONI in plain Z80) ─────────────────────────
        if z == 0:
            if y == 6:          # ED 30: (HL) slot — NONI in eZ80
                result.op = OP.NONI
            else:
                # IN0 r[y], (n)
                result.op = OP.IN0
                result.operands += [TABLE_R[y], (OPER_TYPE.PORT_DEREF, data[1])]
                result.len += 1

        elif z == 1:
            if y == 6:      # ED 31: LD IY,(HL) — repurposes unused OUT0 (n),(HL) slot
                result.op = OP.LD
                result.operands += [(OPER_TYPE.REG, REG.IY), (OPER_TYPE.REG_DEREF, REG.HL)]
            else:
                # OUT0 (n), r[y]
                result.op = OP.OUT0
                result.operands += [(OPER_TYPE.PORT_DEREF, data[1]), TABLE_R[y]]
                result.len += 1

        elif z == 2 and q == 0:
            # LEA rp, IX+d   (only even y: p→BC/DE/HL/IX)
            result.op = OP.LEA
            result.operands += [TABLE_LEA_IX[p], (OPER_TYPE.DISPL_IX, int8(data[1]))]
            result.len += 1

        elif z == 3 and q == 0:
            # LEA rp, IY+d   (only even y: p→BC/DE/HL/IY)
            result.op = OP.LEA
            result.operands += [TABLE_LEA_IY[p], (OPER_TYPE.DISPL_IY, int8(data[1]))]
            result.len += 1

        elif z == 4:
            # TST A, r[y]
            result.op = OP.TST
            result.operands += [(OPER_TYPE.REG, REG.A), TABLE_R[y]]

        elif z == 6 and y == 7:     # ED 3E: LD (HL),IY
            result.op = OP.LD
            result.operands += [(OPER_TYPE.REG_DEREF, REG.HL), (OPER_TYPE.REG, REG.IY)]

        elif z == 7:
            if q == 0:              # LD rr,(HL): p=0→BC, 1→DE, 2→HL, 3→IX
                result.op = OP.LD
                result.operands += [TABLE_LEA_IX[p], (OPER_TYPE.REG_DEREF, REG.HL)]
            else:                   # LD (HL),rr: p=0→BC, 1→DE, 2→HL, 3→IX
                result.op = OP.LD
                result.operands += [(OPER_TYPE.REG_DEREF, REG.HL), TABLE_LEA_IX[p]]

        else:
            result.op = OP.NONI

    elif x == 1:
        # ── Standard Z80 ED instructions, extended at specific slots ────────
        if z == 0:      # IN r[y]/(F), (C)
            result.op = OP.IN
            result.operands.append(TABLE_R[y] if y != 6 else (OPER_TYPE.REG, REG.F))
            result.operands.append((OPER_TYPE.REG_DEREF, REG.C))

        elif z == 1:    # OUT (C), r[y]/0
            result.op = OP.OUT
            result.operands.append((OPER_TYPE.REG_DEREF, REG.C))
            result.operands.append(TABLE_R[y] if y != 6 else (OPER_TYPE.IMM, 0))

        elif z == 2:    # SBC/ADC HL, rp
            result.op = OP.ADC if q else OP.SBC
            result.operands += [(OPER_TYPE.REG, REG.HL), TABLE_RP[p]]

        elif z == 3:    # LD (nn), rp  /  LD rp, (nn)
            addr_val, nb = load_imm(data, 1, long_op)
            result.op = OP.LD
            result.operands += [(OPER_TYPE.ADDR_DEREF, addr_val), TABLE_RP[p]]
            result.len += nb
            if q:
                reorder(result)

        elif z == 4:
            # NEG (y=0), LEA IX,IY+d (y=2), TST A,n (y=4), TSTIO n (y=6), MLT rp (q=1)
            if q == 1:
                # MLT: y=1→BC, y=3→DE, y=5→HL, y=7→SP
                result.op = OP.MLT
                result.operands.append(TABLE_RP[p])
            elif y == 2:
                # LEA IX,IY+d  (ED 54)
                result.op = OP.LEA
                result.operands += [(OPER_TYPE.REG, REG.IX), (OPER_TYPE.DISPL_IY, int8(data[1]))]
                result.len += 1
            elif y == 4:
                # TST A, n  (ED 64)
                result.op = OP.TST
                result.operands += [(OPER_TYPE.REG, REG.A), (OPER_TYPE.IMM, data[1])]
                result.len += 1
            elif y == 6:
                # TSTIO n  (ED 74)
                result.op = OP.TSTIO
                result.operands.append((OPER_TYPE.IMM, data[1]))
                result.len += 1
            else:
                result.op = OP.NEG

        elif z == 5:
            # RETI (y=1), LEA IY,IX+d (y=2), PEA IX+d (y=4), LD MB,A (y=5), STMIX (y=7)
            if y == 1:
                result.op = OP.RETI
            elif y == 2:
                # LEA IY,IX+d  (ED 55)
                result.op = OP.LEA
                result.operands += [(OPER_TYPE.REG, REG.IY), (OPER_TYPE.DISPL_IX, int8(data[1]))]
                result.len += 1
            elif y == 4:
                # PEA IX+d  (ED 65)
                result.op = OP.PEA
                result.operands.append((OPER_TYPE.DISPL_IX, int8(data[1])))
                result.len += 1
            elif y == 5:
                # LD MB,A  (ED 6D)
                result.op = OP.LD
                result.operands += [(OPER_TYPE.REG, REG.MB), (OPER_TYPE.REG, REG.A)]
            elif y == 7:
                result.op = OP.STMIX    # ED 7D
            else:
                result.op = OP.RETN

        elif z == 6:
            # IM (Z80), PEA IY+d (y=4), LD A,MB (y=5), SLP (y=6), RSMIX (y=7)
            if y == 4:
                # PEA IY+d  (ED 66)
                result.op = OP.PEA
                result.operands.append((OPER_TYPE.DISPL_IY, int8(data[1])))
                result.len += 1
            elif y == 5:
                # LD A,MB  (ED 6E)
                result.op = OP.LD
                result.operands += [(OPER_TYPE.REG, REG.A), (OPER_TYPE.REG, REG.MB)]
            elif y == 6:
                result.op = OP.SLP      # ED 76 — Sleep Mode
            elif y == 7:
                result.op = OP.RSMIX    # ED 7E — Reset Mixed Memory Mode
            else:
                result.op = OP.IM
                result.operands.append((OPER_TYPE.IMM, TABLE_IM[y]))

        elif z == 7:    # LD I/R,A  LD A,I/R  RRD  RLD  NOP
            if y == 0:
                result.op = OP.LD
                result.operands += [(OPER_TYPE.REG, REG.I), (OPER_TYPE.REG, REG.A)]
            elif y == 1:
                result.op = OP.LD
                result.operands += [(OPER_TYPE.REG, REG.R), (OPER_TYPE.REG, REG.A)]
            elif y == 2:
                result.op = OP.LD
                result.operands += [(OPER_TYPE.REG, REG.A), (OPER_TYPE.REG, REG.I)]
            elif y == 3:
                result.op = OP.LD
                result.operands += [(OPER_TYPE.REG, REG.A), (OPER_TYPE.REG, REG.R)]
            elif y == 4:
                result.op = OP.RRD
            elif y == 5:
                result.op = OP.RLD
            else:
                result.op = OP.NOP

    elif x == 2:
        if z <= 3 and y >= 4:
            result.op = TABLE_BLI[y - 4][z]
        elif z == 2 and y <= 3:         # INIM INDM INIMR INDMR
            result.op = (OP.INIM, OP.INDM, OP.INIMR, OP.INDMR)[y]
        elif z == 3 and y <= 3:         # OTIM OTDM OTIMR OTDMR
            result.op = (OP.OTIM, OP.OTDM, OP.OTIMR, OP.OTDMR)[y]
        elif z == 4:                    # INI2 IND2 INI2R IND2R OUTI2 OUTD2 OTI2R OTD2R
            result.op = (OP.INI2, OP.IND2, OP.INI2R, OP.IND2R,
                         OP.OUTI2, OP.OUTD2, OP.OTI2R, OP.OTD2R)[y]
        else:
            result.op = OP.NONI

    elif x == 3:
        # All NONI in Z80; eZ80 adds extended block I/O and LD I/HL at z=7
        if z == 2 and y <= 1:           # ED C2/CA: INIRX, INDRX
            result.op = (OP.INIRX, OP.INDRX)[y]
        elif z == 3 and y <= 1:         # ED C3/CB: OTIRX, OTDRX
            result.op = (OP.OTIRX, OP.OTDRX)[y]
        elif z == 7 and y == 0:     # ED C7: LD I,HL
            result.op = OP.LD
            result.operands += [(OPER_TYPE.REG, REG.I), (OPER_TYPE.REG, REG.HL)]
        elif z == 7 and y == 2:     # ED D7: LD HL,I
            result.op = OP.LD
            result.operands += [(OPER_TYPE.REG, REG.HL), (OPER_TYPE.REG, REG.I)]
        else:
            result.op = OP.NONI


# ---------------------------------------------------------------------------
# Main decode entry point
# ---------------------------------------------------------------------------

def decode(data, addr=0, adl=False):
    """Decode one instruction from data (bytes or list) at address addr.

    adl: True = ADL mode (24-bit immediates/addresses by default).
    eZ80 mode prefix bytes (0x40/0x49/0x52/0x5B) override adl for the
    following instruction and are consumed as part of it.
    """
    result = Decoded()
    if not data:
        return result

    try:
        long_op     = adl
        mode_suffix = ''

        # eZ80 mode prefix bytes (repurpose LD B,B / LD C,C / LD D,D / LD E,E)
        if data[0] in (0x40, 0x49, 0x52, 0x5B):
            pfx      = data[0]
            data     = data[1:]
            result.len = 1
            if pfx == 0x5B:
                mode_suffix = '.lil'; long_op = True
            elif pfx == 0x49:
                mode_suffix = '.lis'; long_op = True
            elif pfx == 0x52:
                mode_suffix = '.sil'; long_op = False
            else:  # 0x40
                mode_suffix = '.sis'; long_op = False

        result.mode_suffix = mode_suffix
        result.long_op     = long_op

        if not data:
            return result

        # Standard prefix detection
        prefix = PREFIX.NONE
        if data[0] == 0xCB:
            prefix = PREFIX.CB
            data   = data[1:]
            result.len += 1
        elif data[0] == 0xED:
            prefix = PREFIX.ED
            data   = data[1:]
            result.len += 1
        elif data[0] in (0xDD, 0xFD):
            if data[1:] and data[1] in (0xDD, 0xED, 0xFD):
                result.len += 1
                result.op = OP.NOP
                return result
            if data[1:] and data[1] == 0xCB:
                prefix = PREFIX.DDCB if data[0] == 0xDD else PREFIX.FDCB
                data   = data[2:]
                result.len += 2
            else:
                prefix = PREFIX.DD if data[0] == 0xDD else PREFIX.FD
                data   = data[1:]
                result.len += 1

        if not data:
            return result

        if prefix == PREFIX.NONE:
            decode_unprefixed(data, addr, result, long_op)

        elif prefix == PREFIX.CB:
            decode_cb(data, addr, result)

        elif prefix == PREFIX.ED:
            decode_ed(data, addr, result, long_op)

        elif prefix in (PREFIX.DD, PREFIX.FD):
            if prefix == PREFIX.DD:
                reg_a, reg_b, reg_c = REG.IX,  REG.IXH, REG.IXL
                ot = OPER_TYPE.MEM_DISPL_IX
            else:
                reg_a, reg_b, reg_c = REG.IY,  REG.IYH, REG.IYL
                ot = OPER_TYPE.MEM_DISPL_IY

            if data[0] in (0x07, 0x17, 0x27, 0x31, 0x37, 0x0F, 0x1F, 0x2F, 0x3E, 0x3F):
                # eZ80 16-bit LD with displacement (not present in plain Z80)
                displ = int8(data[1])
                result.len += 1
                result.op = OP.LD
                other = REG.IY if prefix == PREFIX.DD else REG.IX
                if data[0] in (0x07, 0x17, 0x27):  # LD rr,(reg+d) — BC/DE/HL
                    rr = {0x07: REG.BC, 0x17: REG.DE, 0x27: REG.HL}[data[0]]
                    result.operands += [(OPER_TYPE.REG, rr), (ot, displ)]
                elif data[0] == 0x37:               # LD reg,(reg+d) — same index reg
                    result.operands += [(OPER_TYPE.REG, reg_a), (ot, displ)]
                elif data[0] == 0x31:               # LD other,(reg+d) — other index reg
                    result.operands += [(OPER_TYPE.REG, other), (ot, displ)]
                elif data[0] == 0x3F:               # LD (reg+d),reg — same index reg
                    result.operands += [(ot, displ), (OPER_TYPE.REG, reg_a)]
                elif data[0] == 0x3E:               # LD (reg+d),other — other index reg
                    result.operands += [(ot, displ), (OPER_TYPE.REG, other)]
                else:                               # LD (reg+d),rr  (0x0F/0x1F/0x2F)
                    rr = {0x0F: REG.BC, 0x1F: REG.DE, 0x2F: REG.HL}[data[0]]
                    result.operands += [(ot, displ), (OPER_TYPE.REG, rr)]

            elif will_deref_hl(data[0]):
                if data[0] == 0xE9:     # JP (IX)/(IY) — no displacement
                    displ = 0
                    decode_unprefixed(data, addr, result, long_op)
                else:
                    displ = int8(data[1])
                    result.len += 1
                    decode_unprefixed(data[0:1] + data[2:], addr, result, long_op)
                for i, op in enumerate(result.operands):
                    if op == (OPER_TYPE.REG_DEREF, REG.HL):
                        result.operands[i] = (ot, displ)
            elif data[0] == 0xEB:       # EX DE, HL — not affected by prefix
                decode_unprefixed(data, addr + 1, result, long_op)
            else:
                decode_unprefixed(data, addr + 1, result, long_op)
                for i, op in enumerate(result.operands):
                    if op == (OPER_TYPE.REG, REG.HL): result.operands[i] = (OPER_TYPE.REG, reg_a)
                    if op == (OPER_TYPE.REG, REG.H):  result.operands[i] = (OPER_TYPE.REG, reg_b)
                    if op == (OPER_TYPE.REG, REG.L):  result.operands[i] = (OPER_TYPE.REG, reg_c)

        elif prefix in (PREFIX.DDCB, PREFIX.FDCB):
            displ = int8(data[0])
            result.len += 1
            data = data[1:]
            decode_cb(data, addr, result)
            replacement = OPER_TYPE.MEM_DISPL_IX if prefix == PREFIX.DDCB else OPER_TYPE.MEM_DISPL_IY
            x, y, z, p, q = xyz(data[0])
            if x == 0:
                result.operands[0] = (replacement, displ)
                if z != 6: result.metaLoad = TABLE_R[z]
            elif x == 1:
                result.operands[1] = (replacement, displ)
            else:
                result.operands[1] = (replacement, displ)
                if z != 6: result.metaLoad = TABLE_R[z]

    except IndexError:
        result.op = OP.NONI

    # Classify
    if result.op == OP.NONI:
        result.status = DECODE_STATUS.INVALID_INSTRUCTION
    elif result.op is not None:
        result.status = DECODE_STATUS.OK
        v = result.op.value
        if   v < OP.MARKER_END_LOAD_EXCHANGE.value:         result.typ = INSTRTYPE.LOAD_EXCHANGE
        elif v < OP.MARKER_END_BLOCK_TRANSFER_SEARCH.value: result.typ = INSTRTYPE.BLOCK_TRANSFER_SEARCH
        elif v < OP.MARKER_END_ARITHMETIC_LOGICAL.value:    result.typ = INSTRTYPE.ARITHMETIC_LOGICAL
        elif v < OP.MARKER_END_ROTATE_SHIFT.value:          result.typ = INSTRTYPE.ROTATE_SHIFT
        elif v < OP.MARKER_END_BIT_MANIPULATION.value:      result.typ = INSTRTYPE.BIT_MANIPULATION
        elif v < OP.MARKER_END_JUMP_CALL_RETURN.value:      result.typ = INSTRTYPE.JUMP_CALL_RETURN
        elif v < OP.MARKER_END_INPUT_OUTPUT.value:          result.typ = INSTRTYPE.INPUT_OUTPUT
        elif v < OP.MARKER_END_CPU_CONTROL.value:           result.typ = INSTRTYPE.CPU_CONTROL

    return result


# ---------------------------------------------------------------------------
# String generation
# ---------------------------------------------------------------------------

CC_TO_STR = {
    CC.ALWAYS: '1',
    CC.NOT_N: 'nn', CC.N: 'n',
    CC.NOT_Z: 'nz', CC.Z: 'z',
    CC.NOT_C: 'nc', CC.C: 'c',
    CC.NOT_P: 'po', CC.P: 'pe',
    CC.NOT_S: 'p',  CC.S: 'm',
    CC.NOT_H: 'nh', CC.H: 'h',
}

def _uint2str(d):
    if d == 0: return '0'
    return ('0x%x' % d) if d >= 16 else str(d)

def _displ2str(d):
    if d == 0:   return ''
    if d > 0:    return ('+0x%x' if d >= 16 else '+%d') % d
    return ('-0x%x' if -d >= 16 else '%d') % d

def _reg2str(r):
    name = r.name
    return name if name[-1] != '_' else name[:-1] + "'"

def _oper2str(oper_type, val, long_op=False):
    if oper_type == OPER_TYPE.REG:
        return _reg2str(val)
    elif oper_type == OPER_TYPE.REG_DEREF:
        return '(%s)' % _reg2str(val)
    elif oper_type == OPER_TYPE.ADDR:
        val &= 0xFFFFFF if long_op else 0xFFFF
        return ('0x%06x' if long_op else '0x%04x') % val
    elif oper_type == OPER_TYPE.ADDR_DEREF:
        val &= 0xFFFFFF if long_op else 0xFFFF
        return ('(0x%06x)' if long_op else '(0x%04x)') % val
    elif oper_type == OPER_TYPE.PORT_DEREF:
        return '(0x%02x)' % (val & 0xFF)
    elif oper_type == OPER_TYPE.MEM_DISPL_IX:
        return '(IX%s)' % _displ2str(val)
    elif oper_type == OPER_TYPE.MEM_DISPL_IY:
        return '(IY%s)' % _displ2str(val)
    elif oper_type == OPER_TYPE.DISPL_IX:
        return 'IX%s' % _displ2str(val)
    elif oper_type == OPER_TYPE.DISPL_IY:
        return 'IY%s' % _displ2str(val)
    elif oper_type == OPER_TYPE.IMM:
        return _uint2str(val)
    elif oper_type == OPER_TYPE.COND:
        return CC_TO_STR[val]
    raise ValueError('unknown OPER_TYPE: %s' % oper_type)

def decoded2str(decoded):
    if decoded.status != DECODE_STATUS.OK:
        return '???'

    long_op  = decoded.long_op
    mnemonic = decoded.op.name.lower() + decoded.mode_suffix
    ops      = ','.join(_oper2str(*op, long_op=long_op) for op in decoded.operands)
    result   = (mnemonic + ' ' + ops) if ops else mnemonic

    if decoded.metaLoad:
        ot, ov = decoded.metaLoad
        result = 'ld %s,%s' % (_reg2str(ov), result)

    return result

def disasm(data_or_decoded, pc=0, adl=False):
    """Disassemble one instruction, returning a string.

    data_or_decoded: bytes/list to decode, or an already-decoded Decoded object.
    pc:  program counter (used for relative jump target calculation).
    adl: True = ADL mode (24-bit addresses/immediates by default).
    """
    if isinstance(data_or_decoded, Decoded):
        return decoded2str(data_or_decoded)
    return decoded2str(decode(data_or_decoded, pc, adl))
