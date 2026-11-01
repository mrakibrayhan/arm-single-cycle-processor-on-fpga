#!/usr/bin/env python3
"""
assembler_arm.py - two-pass ARMv4 (ARM state) assembler for the ARMv4-compatible CPUs (ARM-SC, ARM-MC, ARM-P).

    python3 assembler_arm.py prog.s -o program.hex [--depth 128] [--listing prog.lst] [--no-ldr-mov] [--features 0x000]

Input  : GNU-style ARM assembly (the ARM-state subset the cores run).
Output : program.hex = one 8-digit hex word per line, exactly --depth lines (the ROM image read by imem_arm.v through
         $readmemh). An error is reported if the program does not fit.

Instructions
  data processing  AND EOR SUB RSB ADD ADC SBC RSC TST TEQ CMP CMN ORR MOV MVN BIC  (cond, S; operand 2 = #imm, Rm, Rm shift #n,
                   Rm shift Rs, Rm RRX; two-operand short form  add r1, r2 = add r1, r1, r2; pseudo shifts LSL LSR ASR ROR RRX; NOP)
                   unencodable immediates are converted like GNU as does: MOV<->MVN, ADD<->SUB, ADC<->SBC, AND<->BIC, CMP<->CMN
  multiply         MUL MLA UMULL UMLAL SMULL SMLAL
  load / store     LDR STR LDRB STRB LDRT STRT LDRBT STRBT  [Rn] [Rn,#+-imm] [Rn,+-Rm{,shift #n}] with ! ; post-index [Rn],#imm / [Rn],+-Rm{,shift}
                   LDRH STRH LDRSB LDRSH (same modes, imm8 / Rm) ; LDR Rd,label ; LDR Rd,=expr (literal pool)
  block transfer   LDM STM with IA IB DA DB FD FA ED EA, optional !, register lists with ranges, optional ^ ; PUSH POP
  branch           B BL BX  (label, or an absolute address)         SWI / SVC #imm24         ADR Rd,label
Directives: .text .data .bss .section .global .globl .type .size .arm .code .syntax .align .balign .p2align .word .long .int .short
            .hword .2byte .byte .ascii .asciz .string .space .skip .fill .equ .set .ltorg .pool .org .end
Syntax: labels  name:   numeric local labels  1:  with 1b / 1f ; comments  @  or  // ; several statements per line with  ; ;
        expressions: + - * / % << >> & | ^ ~ ( ), decimal / 0x / 0b / 'c' constants, labels, '.' (current address)
Memory: .text is the instruction ROM at 0x00000000 (code, literal pools, constants). .data / .bss labels are placed in the data RAM at
        0x10000000; the RAM is not initialised by program.hex, so only .space / .skip / labels are allowed there.

--no-ldr-mov : LDR Rd,=const always uses a literal-pool word (GNU turns it into MOV/MVN when the constant is a rotated immediate; the base
               core (FEATURES = 0) has no rotated immediates).
--features F : (needs arm_ctrl_model.py, which is not in this repository; normally not used) also check every instruction against the decoder (decode() of arm_ctrl_model.py = decoder_arm.v) and against the classes
               built for FEATURES = F; unsupported or illegal instructions are reported as errors (use  --features 0x7ff  for the full core).
"""
import sys, re, argparse, struct, os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

M32 = 0xFFFFFFFF
RAM_BASE = 0x10000000

CONDS = {"eq": 0, "ne": 1, "cs": 2, "hs": 2, "cc": 3, "lo": 3, "mi": 4, "pl": 5, "vs": 6, "vc": 7,
         "hi": 8, "ls": 9, "ge": 10, "lt": 11, "gt": 12, "le": 13, "al": 14}
REGS = {"r%d" % i: i for i in range(16)}
REGS.update({"sb": 9, "sl": 10, "fp": 11, "ip": 12, "sp": 13, "lr": 14, "pc": 15, "a1": 0, "a2": 1, "a3": 2, "a4": 3,
             "v1": 4, "v2": 5, "v3": 6, "v4": 7, "v5": 8, "v6": 9, "v7": 10, "v8": 11})
DP_OPS = {"and": 0, "eor": 1, "sub": 2, "rsb": 3, "add": 4, "adc": 5, "sbc": 6, "rsc": 7, "tst": 8, "teq": 9, "cmp": 10, "cmn": 11,
          "orr": 12, "mov": 13, "bic": 14, "mvn": 15}
SHIFTS = {"lsl": 0, "lsr": 1, "asr": 2, "ror": 3}
LDM_MODES = {"ia": (0, 1), "ib": (1, 1), "da": (0, 0), "db": (1, 0), "fd": (0, 1), "ed": (1, 1), "fa": (0, 0), "ea": (1, 0)}   # (P, U) for LDM
STM_MODES = {"ia": (0, 1), "ib": (1, 1), "da": (0, 0), "db": (1, 0), "fd": (1, 0), "ed": (0, 0), "fa": (1, 1), "ea": (0, 1)}   # (P, U) for STM


class AsmError(Exception):
    pass


# ---------------------------------------------------------------- mnemonic table
def build_mnemonics():
    """full mnemonic string -> (kind, base, cond, extra).  Both the divided (addeqs) and the unified (addseq) orders are accepted."""
    tab = {}

    def add(name, val):
        if name in tab and tab[name] != val:
            raise RuntimeError("mnemonic collision %s: %s vs %s" % (name, tab[name], val))
        tab[name] = val

    condlist = [""] + list(CONDS)
    for base in DP_OPS:
        for c in condlist:
            add(base + c, ("dp", base, c, 0))
            if base not in ("tst", "teq", "cmp", "cmn"):
                add(base + c + "s", ("dp", base, c, 1))
                add(base + "s" + c, ("dp", base, c, 1))
    for base in ("lsl", "lsr", "asr", "ror", "rrx"):
        for c in condlist:
            add(base + c, ("shift", base, c, 0))
            add(base + c + "s", ("shift", base, c, 1))
            add(base + "s" + c, ("shift", base, c, 1))
    for c in condlist:
        add("nop" + c, ("nop", "nop", c, 0))
    for base in ("mul", "mla", "umull", "umlal", "smull", "smlal"):
        for c in condlist:
            add(base + c, ("mul", base, c, 0))
            add(base + c + "s", ("mul", base, c, 1))
            add(base + "s" + c, ("mul", base, c, 1))
    for base in ("ldr", "str"):
        for c in condlist:
            for sfx in ("", "b", "t", "bt"):
                add(base + c + sfx, ("ls", base, c, sfx))        # divided: ldreqb
                add(base + sfx + c, ("ls", base, c, sfx))        # unified: ldrbeq
            for sfx in ("h", "sb", "sh"):
                if base == "str" and sfx != "h": continue
                add(base + c + sfx, ("lsh", base, c, sfx))
                add(base + sfx + c, ("lsh", base, c, sfx))
    for base in ("ldm", "stm"):
        for c in condlist:
            for m in LDM_MODES:
                add(base + c + m, ("ldm", base, c, m))
                add(base + m + c, ("ldm", base, c, m))
            add(base + c, ("ldm", base, c, "ia"))
    for c in condlist:
        add("push" + c, ("push", "push", c, 0))
        add("pop" + c, ("pop", "pop", c, 0))
        add("b" + c, ("b", "b", c, 0))
        add("bl" + c, ("b", "bl", c, 0))
        add("bx" + c, ("bx", "bx", c, 0))
        add("swi" + c, ("swi", "swi", c, 0))
        add("svc" + c, ("swi", "swi", c, 0))
        add("adr" + c, ("adr", "adr", c, 0))
    return tab


MNEM = build_mnemonics()


# ---------------------------------------------------------------- immediates
def ror32(v, n):
    n &= 31
    return ((v >> n) | (v << (32 - n))) & M32 if n else v & M32


def encode_imm(v):
    """ARM modified immediate: returns (rot4, imm8) with the smallest rotation, or None."""
    v &= M32
    for rot in range(16):
        imm = ror32(v, 32 - 2 * rot) if rot else v          # rotate left by 2*rot
        if imm < 256:
            return rot, imm
    return None


# ---------------------------------------------------------------- expressions
class Expr:
    TOK = re.compile(r"\s*(0[xX][0-9a-fA-F]+|0[bB][01]+|\d+[bf]?|'(?:\\.|[^'])'?|[A-Za-z_.$][\w.$]*|<<|>>|[-+*/%&|^~()])")

    def __init__(self, asm):
        self.asm = asm

    def eval(self, text, loc, final=True):
        """Value of an expression; labels that are not defined yet raise KeyError unless final=False (then 0 is returned)."""
        text = text.strip()
        pos, out = 0, []
        while pos < len(text):
            m = self.TOK.match(text, pos)
            if not m or m.end() == pos:
                raise AsmError("bad expression '%s'" % text)
            t = m.group(1)
            pos = m.end()
            if re.fullmatch(r"\d+[bf]", t):                  # numeric local label reference
                v = self.asm.local_ref(t, loc, final)
                out.append(str(v))
            elif t[0].isdigit():
                out.append(str(int(t, 0)))
            elif t[0] == "'":
                s = t[1:].rstrip("'")
                s = bytes(s, "utf-8").decode("unicode_escape")
                out.append(str(ord(s[0])))
            elif t == ".":
                out.append(str(loc))
            elif re.fullmatch(r"[A-Za-z_.$][\w.$]*", t):
                if t in self.asm.symbols:
                    out.append(str(self.asm.symbols[t]))
                elif final:
                    raise AsmError("undefined symbol '%s'" % t)
                else:
                    out.append("0")
            else:
                out.append({"/": "//"}.get(t, t))
        src = " ".join(out)
        try:
            return int(eval(src, {"__builtins__": {}}, {}))
        except ZeroDivisionError:
            raise AsmError("division by zero in '%s'" % text)
        except Exception:
            raise AsmError("bad expression '%s'" % text)


# ---------------------------------------------------------------- operand helpers
def split_top(s, sep=","):
    out, depth, cur = [], 0, []
    for ch in s:
        if ch in "[{(":
            depth += 1
        elif ch in "]})":
            depth -= 1
        if ch == sep and depth == 0:
            out.append("".join(cur).strip())
            cur = []
        else:
            cur.append(ch)
    last = "".join(cur).strip()
    if last or out:
        out.append(last)
    return out


def parse_reg(s):
    s = s.strip().lower()
    if s in REGS:
        return REGS[s]
    raise AsmError("expected a register, got '%s'" % s)


def is_reg(s):
    return s.strip().lower() in REGS


class Item:
    """One thing in the output: an instruction (4 bytes, resolved in pass 2) or raw data bytes."""
    def __init__(self, sec, addr, kind, src, lineno, **kw):
        self.sec, self.addr, self.kind, self.src, self.lineno = sec, addr, kind, src, lineno
        self.__dict__.update(kw)
        self.size = kw.get("size", 4)


class Assembler:
    def __init__(self, ldr_mov=True, features=None):
        self.symbols = {}
        self.abs_syms = set()           # symbols that are absolute constants (.equ/.set of constants), not addresses
        self.items = []
        self.ex = Expr(self)
        self.sec = "text"
        self.loc = {"text": 0, "data": RAM_BASE}
        self.ldr_mov = ldr_mov
        self.features = features
        self.pending_lits = []          # (item, value-expression text) waiting for a pool
        self.pool_items = []
        self.locals = {}                # numeric label -> list of addresses
        self.errors = []
        self.lineno = 0

    # ------------- local numeric labels
    def local_ref(self, t, loc, final):
        n, d = t[:-1], t[-1]
        lst = self.locals.get(n, [])
        if d == "b":
            c = [a for a in lst if a <= loc]
            if not c:
                raise AsmError("no earlier local label %s" % n)
            return c[-1]
        c = [a for a in lst if a > loc]
        if not c:
            if final:
                raise AsmError("no later local label %s" % n)
            return 0
        return c[0]

    # ------------- driver
    def assemble(self, text):
        for no, raw in enumerate(text.splitlines(), 1):
            self.lineno = no
            try:
                self.line(raw)
            except AsmError as e:
                self.errors.append("line %d: %s    [%s]" % (no, e, raw.strip()))
        if not self.errors:
            self.flush_pool(final=True)
            self.resolve()
        return self.errors

    @staticmethod
    def strip_comment(s):
        out, q = [], None
        i = 0
        while i < len(s):
            ch = s[i]
            if q:
                out.append(ch)
                if ch == "\\":
                    i += 1
                    if i < len(s): out.append(s[i])
                elif ch == q:
                    q = None
            else:
                if ch in "\"'":
                    q = ch
                    out.append(ch)
                elif ch == "@" or (ch == "/" and s[i:i + 2] == "//"):
                    break
                else:
                    out.append(ch)
            i += 1
        return "".join(out)

    def line(self, raw):
        s = self.strip_comment(raw).strip()
        if s.startswith("#") and not s.startswith("#include"):
            return                                              # cpp-style comment line
        # statements separated by ';' (outside quotes)
        for stmt in self.split_stmts(s):
            self.statement(stmt)

    @staticmethod
    def split_stmts(s):
        out, cur, q = [], [], None
        for ch in s:
            if q:
                cur.append(ch)
                if ch == q: q = None
            elif ch in "\"'":
                q = ch; cur.append(ch)
            elif ch == ";":
                out.append("".join(cur)); cur = []
            else:
                cur.append(ch)
        out.append("".join(cur))
        return [x.strip() for x in out if x.strip()]

    def statement(self, s):
        while True:                                             # labels
            m = re.match(r"^([A-Za-z_.$][\w.$]*|\d+)\s*:\s*(.*)$", s)
            if not m:
                break
            self.define_label(m.group(1))
            s = m.group(2)
        if not s:
            return
        if s[0] == ".":
            return self.directive(s)
        m = re.match(r"^(\S+)\s*(.*)$", s)
        mn, ops = m.group(1).lower(), m.group(2).strip()
        self.instruction(mn, ops, s)

    def define_label(self, name):
        if name.isdigit():
            self.locals.setdefault(name, []).append(self.loc[self.sec])
            return
        if name in self.symbols:
            raise AsmError("symbol '%s' defined twice" % name)
        self.symbols[name] = self.loc[self.sec]

    # ------------- emitting
    def emit(self, kind, src, **kw):
        if self.sec != "text":
            raise AsmError("instructions and initialised data are only allowed in .text (the RAM is not initialised by program.hex)")
        it = Item(self.sec, self.loc[self.sec], kind, src, self.lineno, **kw)
        self.items.append(it)
        self.loc[self.sec] += it.size
        return it

    def align(self, n, fill=None):
        """fill=None: like GNU as, padding after an instruction is NOPs (mov r0,r0), after data it is zero bytes."""
        pad = (-self.loc[self.sec]) % n
        if pad:
            if self.sec == "text":
                last = self.items[-1] if self.items else None
                if fill is None and last is not None and last.kind == "ins" and pad % 4 == 0:
                    data = struct.pack("<I", 0xE1A00000) * (pad // 4)
                else:
                    data = bytes([(fill or 0) & 255]) * pad
                self.emit("data", ".align", data=data, size=pad)
            else:
                self.loc[self.sec] += pad

    # ------------- directives
    def directive(self, s):
        m = re.match(r"^(\.\w+)\s*(.*)$", s)
        d, rest = m.group(1).lower(), m.group(2).strip()
        ex = self.ex
        if d in (".text",):
            self.sec = "text"
        elif d in (".data", ".bss"):
            self.sec = "data"
        elif d == ".section":
            name = rest.split(",")[0].strip()
            self.sec = "data" if (name.startswith(".data") or name.startswith(".bss") or name.startswith(".sdata")) else "text"
        elif d in (".global", ".globl", ".type", ".size", ".arm", ".syntax", ".file", ".ident", ".eabi_attribute", ".fpu", ".cpu",
                   ".arch", ".weak", ".local", ".comm", ".end", ".code", ".extern", ".func", ".endfunc", ".fnstart", ".fnend"):
            if d == ".code" and rest.strip() not in ("32", ""):
                raise AsmError("Thumb (.code 16) is not supported")
            return
        elif d in (".thumb", ".thumb_func"):
            raise AsmError("Thumb is not supported")
        elif d in (".align", ".p2align", ".balign"):
            parts = [p for p in split_top(rest) if p]
            n = ex.eval(parts[0], self.loc[self.sec]) if parts else 0
            fill = ex.eval(parts[1], self.loc[self.sec]) if len(parts) > 1 and parts[1] else None
            self.align((1 << n) if d != ".balign" else n, fill)
        elif d in (".word", ".long", ".int", ".4byte"):
            self.align(4) if d != ".4byte" else None
            for e in split_top(rest):
                self.emit("word", s, expr=e, size=4)
        elif d in (".short", ".hword", ".2byte", ".half"):
            self.align(2) if d != ".2byte" else None
            for e in split_top(rest):
                self.emit("half", s, expr=e, size=2)
        elif d == ".byte":
            for e in split_top(rest):
                self.emit("byte", s, expr=e, size=1)
        elif d in (".ascii", ".asciz", ".string"):
            for e in split_top(rest):
                mm = re.fullmatch(r'"((?:\\.|[^"\\])*)"', e.strip())
                if not mm:
                    raise AsmError("bad string")
                b = bytes(mm.group(1), "utf-8").decode("unicode_escape").encode("latin-1")
                if d != ".ascii": b += b"\0"
                self.emit("data", s, data=b, size=len(b))
        elif d in (".space", ".skip", ".zero"):
            parts = split_top(rest)
            n = ex.eval(parts[0], self.loc[self.sec])
            fill = ex.eval(parts[1], self.loc[self.sec]) if len(parts) > 1 else 0
            if self.sec == "text":
                self.emit("data", s, data=bytes([fill & 255]) * n, size=n)
            else:
                if fill: raise AsmError("the RAM is not initialised by program.hex")
                self.loc[self.sec] += n
        elif d == ".fill":
            parts = split_top(rest)
            cnt = ex.eval(parts[0], self.loc[self.sec])
            size = ex.eval(parts[1], self.loc[self.sec]) if len(parts) > 1 else 1
            val = ex.eval(parts[2], self.loc[self.sec]) if len(parts) > 2 else 0
            if self.sec != "text": raise AsmError("the RAM is not initialised by program.hex")
            self.emit("data", s, data=(val & ((1 << 8 * size) - 1)).to_bytes(size, "little") * cnt, size=size * cnt)
        elif d in (".equ", ".set"):
            parts = split_top(rest)
            if len(parts) != 2: raise AsmError("%s needs name, value" % d)
            self.symbols[parts[0]] = ex.eval(parts[1], self.loc[self.sec])
            if self.is_const(parts[1]):
                self.abs_syms.add(parts[0])
            else:
                self.abs_syms.discard(parts[0])
        elif d in (".ltorg", ".pool"):
            self.flush_pool()
        elif d == ".org":
            parts = split_top(rest)
            target = ex.eval(parts[0], self.loc[self.sec]) + (RAM_BASE if False else 0)
            fill = ex.eval(parts[1], self.loc[self.sec]) if len(parts) > 1 else 0
            cur = self.loc[self.sec]
            if target < cur: raise AsmError(".org moves backwards")
            if self.sec == "text":
                self.emit("data", s, data=bytes([fill & 255]) * (target - cur), size=target - cur)
            else:
                self.loc[self.sec] = target
        else:
            raise AsmError("unknown directive %s" % d)

    def is_const(self, e):
        """True when the expression is a pure constant (numbers and absolute .equ symbols only: no labels, no '.', no local labels)."""
        t = re.sub(r"0[xX][0-9a-fA-F]+|0[bB][01]+|'(?:\\.|[^'])'?", " ", e)
        if re.search(r"\d+[bf]\b", t) or re.search(r"(?<![\w.$])\.(?![\w.$])", t):
            return False
        names = re.findall(r"[A-Za-z_.$][\w.$]*", t)
        return all(n in self.abs_syms for n in names)

    # ------------- literal pool
    def flush_pool(self, final=False):
        lits = self.pending_lits
        if not lits:
            return
        self.pending_lits = []
        self.align(4)
        seen = {}
        for it in lits:
            key = it.litkey
            if key in seen:
                it.pool_addr = seen[key]
                continue
            pit = self.emit("word", "literal pool", expr=it.litexpr, size=4, is_pool=True)
            seen[key] = pit.addr
            it.pool_addr = pit.addr

    # ------------- instructions
    def instruction(self, mn, ops, src):
        if mn not in MNEM:
            raise AsmError("unknown instruction '%s'" % mn)
        kind, base, cond, extra = MNEM[mn]
        c = CONDS[cond] if cond else 14
        if kind == "dp": return self.i_dp(src, base, c, extra, ops)
        if kind == "shift": return self.i_shift(src, base, c, extra, ops)
        if kind == "nop": return self.emit("ins", src, fn=lambda: (c << 28) | 0x01A00000)
        if kind == "mul": return self.i_mul(src, base, c, extra, ops)
        if kind == "ls": return self.i_ls(src, base, c, extra, ops)
        if kind == "lsh": return self.i_lsh(src, base, c, extra, ops)
        if kind == "ldm": return self.i_ldm(src, base, c, extra, ops)
        if kind in ("push", "pop"):
            lst = ops.strip()
            single = re.fullmatch(r"\{\s*([A-Za-z0-9]+)\s*\}", lst)
            if single and is_reg(single.group(1)):                # GNU as: push {rX} = str rX,[sp,#-4]! ; pop {rX} = ldr rX,[sp],#4
                r = single.group(1)
                if kind == "push":
                    return self.i_ls(src, "str", c, "", "%s, [sp, #-4]!" % r)
                return self.i_ls(src, "ldr", c, "", "%s, [sp], #4" % r)
            if kind == "push": return self.i_ldm(src, "stm", c, "db", "sp!, " + ops)
            return self.i_ldm(src, "ldm", c, "ia", "sp!, " + ops)
        if kind == "b": return self.i_branch(src, base, c, ops)
        if kind == "bx":
            rm = parse_reg(ops)
            return self.emit("ins", src, fn=lambda: (c << 28) | 0x012FFF10 | rm)
        if kind == "swi":
            e = ops.lstrip("#").strip()
            def swi_word():
                v = self.ex.eval(e or "0", self.cur)
                if not 0 <= v <= 0xFFFFFF: raise AsmError("SWI number 0x%x does not fit in 24 bits" % v)
                return (c << 28) | 0x0F000000 | v
            return self.emit("ins", src, fn=swi_word)
        if kind == "adr": return self.i_adr(src, c, ops)

    # operand 2 -------------------------------------------------
    def parse_op2(self, parts):
        """returns a function (item_loc) -> ('imm', value) or ('reg', bits12 encoding, I=0) ; value for immediates evaluated late"""
        if not parts:
            raise AsmError("missing operand")
        p0 = parts[0].strip()
        if p0.startswith("#"):
            if len(parts) == 2 and not parts[1].strip().startswith("#"):
                return ("immrot", p0[1:].strip(), parts[1].strip())      # #imm8, rot   (explicit even rotation, as objdump prints it)
            if len(parts) != 1:
                raise AsmError("too many operands after the immediate")
            return ("imm", p0[1:].strip())
        rm = parse_reg(p0)
        if len(parts) == 1:
            return ("reg", rm, 0, 0, None)
        sh = parts[1].strip().lower()
        if len(parts) > 2:
            raise AsmError("too many operands")
        if sh == "rrx":
            return ("reg", rm, 3, 0, None)
        m = re.match(r"^(lsl|lsr|asr|ror)\s+(.*)$", sh)
        if not m:
            raise AsmError("bad shift '%s'" % sh)
        typ, amt = SHIFTS[m.group(1)], m.group(2).strip()
        if amt.startswith("#"):
            return ("reg", rm, typ, 1, amt[1:].strip())
        if is_reg(amt):
            return ("reg", rm, typ, 2, parse_reg(amt))
        raise AsmError("bad shift amount '%s'" % amt)

    def shifter_bits(self, op2, loc):
        """encoding of bits 11..0 for a register operand 2"""
        _, rm, typ, mode, amt = op2
        if mode == 0:
            if typ == 3:                                         # RRX = ROR #0
                return (3 << 5) | rm
            return rm
        if mode == 2:
            return (amt << 8) | (typ << 5) | (1 << 4) | rm
        n = self.ex.eval(amt, loc)
        if typ == 0:
            if not 0 <= n <= 31: raise AsmError("LSL amount %d out of range 0-31" % n)
        elif typ in (1, 2):
            if not 0 <= n <= 32: raise AsmError("%s amount %d out of range 1-32" % ("LSR" if typ == 1 else "ASR", n))
            if n == 0:
                return rm                                        # LSR #0 / ASR #0 are encoded as LSL #0 (like GNU as)
            n &= 31                                              # 32 is encoded as 0
        else:
            if n == 0:
                return rm                                        # ROR #0 = no shift (RRX needs the RRX keyword)
            if not 1 <= n <= 31: raise AsmError("ROR amount %d out of range 1-31" % n)
        return (n << 7) | (typ << 5) | rm

    def dp_word(self, base, c, s, rd, rn, op2, loc):
        opc = DP_OPS[base]
        if op2[0] == "immrot":
            v8, rot = self.ex.eval(op2[1], loc), self.ex.eval(op2[2], loc)
            if not (0 <= v8 <= 255 and rot % 2 == 0 and 0 <= rot <= 30):
                raise AsmError("#imm8, rot needs 0 <= imm8 <= 255 and an even rotation 0-30")
            return (c << 28) | (1 << 25) | (opc << 21) | (s << 20) | (rn << 16) | (rd << 12) | ((rot // 2) << 8) | v8
        if op2[0] == "imm":
            v = self.ex.eval(op2[1], loc) & M32
            if rn == 15 and base == "add" and not s and v >= 1 << 31:
                base, opc, v = "sub", DP_OPS["sub"], (-v) & M32      # GNU as: ADD Rd,PC,#negative becomes SUB Rd,PC,#-value (no fallback)
                if encode_imm(v) is None: raise AsmError("immediate cannot be encoded")
            e = encode_imm(v)
            if e is None:
                alt = {"mov": ("mvn", (~v) & M32), "mvn": ("mov", (~v) & M32), "add": ("sub", (-v) & M32), "sub": ("add", (-v) & M32),
                       "cmp": ("cmn", (-v) & M32), "cmn": ("cmp", (-v) & M32), "and": ("bic", (~v) & M32), "bic": ("and", (~v) & M32),
                       "adc": ("sbc", (~v) & M32), "sbc": ("adc", (~v) & M32)}.get(base)
                if alt and encode_imm(alt[1]) is not None:
                    opc = DP_OPS[alt[0]]
                    e = encode_imm(alt[1])
                else:
                    raise AsmError("immediate 0x%x cannot be encoded as a rotated 8-bit value" % v)
            rot, imm = e
            return (c << 28) | (1 << 25) | (opc << 21) | (s << 20) | (rn << 16) | (rd << 12) | (rot << 8) | imm
        return (c << 28) | (opc << 21) | (s << 20) | (rn << 16) | (rd << 12) | self.shifter_bits(op2, loc)

    def i_dp(self, src, base, c, s, ops):
        parts = split_top(ops)
        if base in ("tst", "teq", "cmp", "cmn"):
            if len(parts) < 2: raise AsmError("%s needs 2 operands" % base)
            rn = parse_reg(parts[0]); op2 = self.parse_op2(parts[1:])
            return self.emit("ins", src, fn=lambda: self.dp_word(base, c, 1, 0, rn, op2, self.cur))
        if base in ("mov", "mvn"):
            if len(parts) < 2: raise AsmError("%s needs 2 operands" % base)
            rd = parse_reg(parts[0]); op2 = self.parse_op2(parts[1:])
            return self.emit("ins", src, fn=lambda: self.dp_word(base, c, s, rd, 0, op2, self.cur))
        if len(parts) < 2: raise AsmError("%s needs at least 2 operands" % base)
        rd = parse_reg(parts[0])
        if len(parts) == 2 or (len(parts) == 3 and re.match(r"^(lsl|lsr|asr|ror|rrx)\b", parts[2].strip().lower())):
            rn, rest = rd, parts[1:]                              # add r1, r2   ==  add r1, r1, r2
        else:
            rn, rest = parse_reg(parts[1]), parts[2:]
        op2 = self.parse_op2(rest)
        return self.emit("ins", src, fn=lambda: self.dp_word(base, c, s, rd, rn, op2, self.cur))

    def i_shift(self, src, base, c, s, ops):
        parts = split_top(ops)
        if base == "rrx":
            if len(parts) == 2: parts = [parts[0], parts[1]]
            rd, rm = parse_reg(parts[0]), parse_reg(parts[1])
            op2 = ("reg", rm, 3, 0, None)
        else:
            if len(parts) == 2:                                   # lsl r1, #3  ==  lsl r1, r1, #3
                parts = [parts[0], parts[0], parts[1]]
            if len(parts) != 3: raise AsmError("%s needs 3 operands" % base)
            rd, rm = parse_reg(parts[0]), parse_reg(parts[1])
            amt = parts[2].strip()
            if amt.startswith("#"):
                op2 = ("reg", rm, SHIFTS[base], 1, amt[1:].strip())
            else:
                op2 = ("reg", rm, SHIFTS[base], 2, parse_reg(amt))
        return self.emit("ins", src, fn=lambda: self.dp_word("mov", c, s, rd, 0, op2, self.cur))

    def i_mul(self, src, base, c, s, ops):
        r = [parse_reg(x) for x in split_top(ops)]
        if 15 in r: raise AsmError("PC cannot be used as an operand of %s" % base)
        if base in ("mul",):
            if len(r) == 2: r = [r[0], r[1], r[0]]               # mul rd, rm  ==  mul rd, rm, rd
            if len(r) != 3: raise AsmError("MUL needs rd, rm, rs")
            w = (c << 28) | (s << 20) | (r[0] << 16) | (r[2] << 8) | 0x90 | r[1]
        elif base == "mla":
            if len(r) != 4: raise AsmError("MLA needs rd, rm, rs, rn")
            w = (c << 28) | (1 << 21) | (s << 20) | (r[0] << 16) | (r[3] << 12) | (r[2] << 8) | 0x90 | r[1]
        else:
            if len(r) != 4: raise AsmError("%s needs rdlo, rdhi, rm, rs" % base)
            u = 1 if base.startswith("s") else 0                  # bit 22: 1 = signed
            a = 1 if base.endswith("lal") else 0
            w = (c << 28) | (1 << 23) | (u << 22) | (a << 21) | (s << 20) | (r[1] << 16) | (r[0] << 12) | (r[3] << 8) | 0x90 | r[2]
        return self.emit("ins", src, fn=lambda: w)

    # memory ------------------------------------------------------
    def parse_addr(self, parts, half):
        """returns dict(rn, p, w, u, off) with off = ('imm', expr) or ('reg', rm, shiftinfo) or None"""
        a = parts[0].strip()
        m = re.match(r"^\[\s*([^,\]]+)\s*(?:,\s*(.*?))?\s*\](!?)$", a)
        if not m:
            raise AsmError("bad address '%s'" % a)
        rn = parse_reg(m.group(1))
        inner, bang = m.group(2), m.group(3) == "!"
        if rn == 15 and (bang or len(parts) > 1):
            raise AsmError("write-back / post-index with PC as base register is not allowed")
        if len(parts) > 1:                                         # post-index
            if inner is not None or bang: raise AsmError("bad post-index address")
            return dict(rn=rn, p=0, w=0, u=None, off=self.parse_off(parts[1:], half))
        if inner is None:
            return dict(rn=rn, p=1, w=1 if bang else 0, u=1, off=("imm", "0"))
        return dict(rn=rn, p=1, w=1 if bang else 0, u=None, off=self.parse_off(split_top(inner), half))

    def parse_off(self, parts, half):
        p0 = parts[0].strip()
        if p0.startswith("#"):
            if len(parts) > 1: raise AsmError("bad offset")
            return ("imm", p0[1:].strip())
        neg = 0
        if p0.startswith("-"): neg, p0 = 1, p0[1:]
        elif p0.startswith("+"): p0 = p0[1:]
        rm = parse_reg(p0)
        if rm == 15: raise AsmError("PC cannot be the offset register")
        if len(parts) == 1:
            return ("reg", rm, None, neg)
        if half: raise AsmError("halfword addressing cannot shift the register")
        m = re.match(r"^(lsl|lsr|asr|ror)\s+#(.*)$", parts[1].strip().lower())
        if parts[1].strip().lower() == "rrx":
            return ("reg", rm, (3, None), neg)
        if not m: raise AsmError("bad shift in address")
        return ("reg", rm, (SHIFTS[m.group(1)], m.group(2).strip()), neg)

    def i_ls(self, src, base, c, sfx, ops):
        load = 1 if base == "ldr" else 0
        b = 1 if "b" in sfx else 0
        t = 1 if "t" in sfx else 0
        parts = split_top(ops)
        if len(parts) < 2: raise AsmError("%s needs a register and an address" % base)
        rd = parse_reg(parts[0])
        if rd == 15 and (b or (t and load)): raise AsmError("%s cannot use PC as the data register" % (base + sfx))
        rest = parts[1:]
        if not rest[0].startswith("["):
            if not load: raise AsmError("STR needs a [..] address")
            # LDR rd, label   or   LDR rd, =expr
            e = rest[0].strip()
            if e.startswith("="):
                return self.ldr_literal(src, c, rd, b, e[1:].strip())
            return self.emit("ins", src, fn=lambda: self.ldr_pcrel(c, rd, b, self.ex.eval(e, self.cur), self.cur))
        ad = self.parse_addr(rest, False)
        if not load and rd == 15 and ad["rn"] == 15: raise AsmError("STR PC,[PC,..] is unpredictable")
        if t and ad["p"] == 1:
            if ad["off"] == ("imm", "0") and ad["w"] == 0 and rest[0].replace(" ", "").endswith("]") and "," not in rest[0]:
                ad["p"] = 0                                        # ldrt rd,[rn]  ==  ldrt rd,[rn],#0
            else:
                raise AsmError("%s needs post-index addressing" % base)
        if t: ad["w"] = 1
        w = ad["w"]
        if ad["p"] == 0 and not t: w = 0

        def build():
            off = ad["off"]
            if off[0] == "imm":
                v = self.ex.eval(off[1], self.cur)
                u = 1 if (v > 0 or (v == 0 and not off[1].strip().startswith("-"))) else 0     # '#-0' sets U=0 like GNU as
                if abs(v) > 4095: raise AsmError("offset %d out of range (-4095..4095)" % v)
                if load and rd == 15 and ad["rn"] == 15 and v % 4: raise AsmError("LDR PC,[PC,#n] needs a word-aligned offset")
                low = abs(v)
                i = 0
            else:
                _, rm, sh, neg = off
                u = 0 if neg else 1
                i = 1
                if sh is None: low = rm
                elif sh[0] == 3 and sh[1] is None: low = (3 << 5) | rm
                else:
                    n = self.ex.eval(sh[1], self.cur)
                    typ = sh[0]
                    if typ == 0 and not 0 <= n <= 31: raise AsmError("LSL amount out of range")
                    if typ in (1, 2):
                        if not 1 <= n <= 32: raise AsmError("shift amount out of range 1-32")
                        n &= 31
                    if typ == 3 and not 1 <= n <= 31: raise AsmError("ROR amount out of range 1-31")
                    low = (n << 7) | (typ << 5) | rm
            return (c << 28) | (1 << 26) | (i << 25) | (ad["p"] << 24) | (u << 23) | (b << 22) | (w << 21) | (load << 20) | (ad["rn"] << 16) | (rd << 12) | low
        return self.emit("ins", src, fn=build)

    def ldr_pcrel(self, c, rd, b, target, loc):
        off = target - (loc + 8)
        u = 1 if off >= 0 else 0
        if abs(off) > 4095: raise AsmError("literal / label too far for LDR (offset %d)" % off)
        return (c << 28) | (0b010 << 25) | (1 << 24) | (u << 23) | (b << 22) | (1 << 20) | (15 << 16) | (rd << 12) | abs(off)

    def ldr_literal(self, src, c, rd, b, e):
        if b: raise AsmError("LDRB rd,=value is not valid")
        key = e.replace(" ", "")
        ldr_mov = self.ldr_mov
        it = None

        def fn():
            if ldr_mov and not it.force_pool:
                v = self.ex.eval(e, self.cur, final=True) & M32
                if encode_imm(v) is not None:
                    return self.dp_word("mov", c, 0, rd, 0, ("imm", str(v)), self.cur)
                if encode_imm(~v & M32) is not None:
                    return self.dp_word("mvn", c, 0, rd, 0, ("imm", str(~v & M32)), self.cur)
            return self.ldr_pcrel(c, rd, 0, it.pool_addr, self.cur)
        it = self.emit("ins", src, fn=fn, litexpr=e, litkey=key, force_pool=False)
        # MOV/MVN form only for pure constants (GNU as keeps address expressions in the pool)
        if ldr_mov and self.is_const(e):
            v = self.ex.eval(e, it.addr) & M32
            if encode_imm(v) is not None or encode_imm(~v & M32) is not None:
                return it
        it.force_pool = True
        self.pending_lits.append(it)
        return it

    def i_lsh(self, src, base, c, sfx, ops):
        load = 1 if base == "ldr" else 0
        sh = {"h": 1, "sb": 2, "sh": 3}[sfx]
        parts = split_top(ops)
        if len(parts) < 2 or not parts[1].startswith("["): raise AsmError("%s%s needs a register and a [..] address" % (base, sfx))
        rd = parse_reg(parts[0])
        if rd == 15: raise AsmError("%s%s cannot use PC as the data register" % (base, sfx))
        ad = self.parse_addr(parts[1:], True)

        def build():
            off = ad["off"]
            if off[0] == "imm":
                v = self.ex.eval(off[1], self.cur)
                u = 1 if (v > 0 or (v == 0 and not off[1].strip().startswith("-"))) else 0
                if abs(v) > 255: raise AsmError("halfword offset %d out of range (-255..255)" % v)
                low = ((abs(v) >> 4) << 8) | (abs(v) & 15)
                ibit = 1
            else:
                _, rm, shf, neg = off
                u = 0 if neg else 1
                low = rm
                ibit = 0
            return (c << 28) | (ad["p"] << 24) | (u << 23) | (ibit << 22) | (ad["w"] << 21) | (load << 20) | (ad["rn"] << 16) | (rd << 12) | low | 0x90 | (sh << 5)
        return self.emit("ins", src, fn=build)

    def i_ldm(self, src, base, c, mode, ops):
        parts = split_top(ops)
        if len(parts) < 2: raise AsmError("%s needs a base register and a register list" % base)
        bs = parts[0].strip()
        w = 1 if bs.endswith("!") else 0
        rn = parse_reg(bs.rstrip("!"))
        if rn == 15: raise AsmError("PC cannot be the base register of %s" % base)
        lst = ",".join(parts[1:]).strip()
        s = 1 if lst.endswith("^") else 0
        lst = lst.rstrip("^").strip()
        if not (lst.startswith("{") and lst.endswith("}")): raise AsmError("bad register list")
        mask = 0
        for r in split_top(lst[1:-1]):
            if not r: continue
            if "-" in r:
                a, b2 = [parse_reg(x) for x in r.split("-")]
                if b2 < a: raise AsmError("bad register range")
                for k in range(a, b2 + 1): mask |= 1 << k
            else:
                mask |= 1 << parse_reg(r)
        if mask == 0: raise AsmError("empty register list")
        tab = LDM_MODES if base == "ldm" else STM_MODES
        p, u = tab[mode]
        load = 1 if base == "ldm" else 0
        w_ = (c << 28) | (0b100 << 25) | (p << 24) | (u << 23) | (s << 22) | (w << 21) | (load << 20) | (rn << 16) | mask
        return self.emit("ins", src, fn=lambda: w_)

    def i_branch(self, src, base, c, ops):
        link = 1 if base == "bl" else 0
        e = ops.strip()

        def build():
            tgt = self.ex.eval(e, self.cur)
            d = (tgt - (self.cur + 8)) & M32                       # 32-bit wrap-around like the hardware PC
            if d >= 1 << 31: d -= 1 << 32
            if d % 4: raise AsmError("branch target 0x%x is not word aligned" % tgt)
            d //= 4
            if not -(1 << 23) <= d < (1 << 23): raise AsmError("branch target out of range")
            return (c << 28) | (0b101 << 25) | (link << 24) | (d & 0xFFFFFF)
        return self.emit("ins", src, fn=build)

    def i_adr(self, src, c, ops):
        parts = split_top(ops)
        if len(parts) != 2: raise AsmError("ADR needs rd, label")
        rd = parse_reg(parts[0]); e = parts[1].strip()

        def build():
            tgt = self.ex.eval(e, self.cur)
            off = tgt - (self.cur + 8)
            if off >= 0:
                base, v = "add", off
            else:
                base, v = "sub", -off
            if encode_imm(v) is None: raise AsmError("ADR target out of range / not encodable")
            return self.dp_word(base, c, 0, rd, 15, ("imm", str(v)), self.cur)
        return self.emit("ins", src, fn=build)

    # ------------- pass 2
    def resolve(self):
        for lst in self.locals.values():
            lst.sort()
        for it in self.items:
            self.cur = it.addr
            self.lineno = it.lineno
            try:
                if it.kind == "ins":
                    it.word = it.fn() & M32
                    it.bytes = struct.pack("<I", it.word)
                elif it.kind == "word":
                    it.bytes = struct.pack("<I", self.ex.eval(it.expr, it.addr) & M32)
                elif it.kind == "half":
                    it.bytes = struct.pack("<H", self.ex.eval(it.expr, it.addr) & 0xFFFF)
                elif it.kind == "byte":
                    it.bytes = bytes([self.ex.eval(it.expr, it.addr) & 0xFF])
                else:
                    it.bytes = it.data
            except AsmError as e:
                self.errors.append("line %d: %s    [%s]" % (it.lineno, e, it.src))

    def image(self):
        """bytes of .text starting at address 0"""
        end = max((it.addr + it.size for it in self.items), default=0)
        buf = bytearray(end)
        for it in self.items:
            buf[it.addr:it.addr + it.size] = it.bytes
        return bytes(buf)


# ---------------------------------------------------------------- feature / decoder check
def check_features(asm, features):
    import arm_ctrl_model as cm
    import arm_features as af
    msgs = []
    for it in asm.items:
        if it.kind != "ins" or getattr(it, "is_pool", False):
            continue
        w = it.word
        g = cm.decode(w)
        if g["illegal"]:
            msgs.append("line %d: 0x%08x is rejected by the decoder (%s)    [%s]" % (it.lineno, w, g.get("why", "illegal"), it.src))
            continue
        need = [i for i in af.needed_feature(w) if not (features >> i) & 1]
        if need:
            msgs.append("line %d: 0x%08x needs feature(s) not in FEATURES=0x%03x: %s    [%s]" % (
                it.lineno, w, features, "; ".join(af.FEATURE_NAMES[i] for i in need), it.src))
    return msgs


def write_hex(path, image, depth):
    words = list(struct.unpack("<%dI" % (len(image) // 4), image + b"\0" * (-len(image) % 4)))
    if len(words) > depth:
        raise AsmError("program is %d words, the ROM has %d" % (len(words), depth))
    words += [0] * (depth - len(words))
    with open(path, "w") as f:
        f.write("\n".join("%08x" % w for w in words) + "\n")


def main():
    ap = argparse.ArgumentParser(description="ARMv4 assembler for the single-cycle ARM CPU")
    ap.add_argument("source")
    ap.add_argument("-o", "--output", default="program.hex")
    ap.add_argument("--depth", type=int, default=128, help="ROM depth in words (IMEM_AW = 7 -> 128)")
    ap.add_argument("--listing")
    ap.add_argument("--bin")
    ap.add_argument("--no-ldr-mov", action="store_true", help="LDR Rd,=const always uses a literal-pool word")
    ap.add_argument("--features", type=lambda x: int(x, 0), help="check against decoder FEATURES (e.g. 0x000 base core, 0x7ff full core)")
    a = ap.parse_args()
    asm = Assembler(ldr_mov=not a.no_ldr_mov)
    errs = asm.assemble(open(a.source).read())
    if not errs and a.features is not None:
        errs = check_features(asm, a.features)
    if errs:
        print("\n".join(errs), file=sys.stderr)
        print("%d error(s); no output written" % len(errs), file=sys.stderr)
        sys.exit(1)
    img = asm.image()
    try:
        write_hex(a.output, img, a.depth)
    except AsmError as e:
        print("error: %s" % e, file=sys.stderr)
        sys.exit(1)
    if a.bin:
        open(a.bin, "wb").write(img)
    if a.listing:
        with open(a.listing, "w") as f:
            for it in asm.items:
                if it.kind == "ins" or it.kind == "word":
                    f.write("%08x  %08x  %s\n" % (it.addr, struct.unpack("<I", it.bytes)[0], it.src))
                else:
                    f.write("%08x  %-8s  %s\n" % (it.addr, it.bytes.hex()[:8], it.src))
    print("%s: %d bytes (%d words) -> %s (%d words)" % (a.source, len(img), (len(img) + 3) // 4, a.output, a.depth))


if __name__ == "__main__":
    main()
