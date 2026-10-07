"""Keyboard-teachbox — drive the ROB3 firmware's keypad from your keyboard.

An INPUT driver (separate from the viewers): it injects teachbox keypad events
into the running firmware so the arm moves; watch the motion in a viewer
(``rob3-viz``). Two modes:

* **own-engine** — boots its own ucSim and drives it directly with the verified
  debounce cadence (axis-select) + ``kh_jog`` (jog).
* **attach** (``--intent-port N``) — connects to a *running* viewer's INTENT
  socket (opened by ``rob3-viz --intent-port N``) and sends press/jog *intents*.
  The viewer owns the single ucSim stepping loop and applies each intent with the
  verified cadence, then renders — so one sim is shared and jog works live while
  you watch. (There is only ONE stepper: the viewer. The teachbox sends intents.)

Keys (verified ROB3 keypad row/group, from the rob3-firmware-sim skill):

    0-9   numeric (POSITION mode: keys 2..7 select axis 0..5)
    + -   jog selected axis up / down
    P POS   E ENT   N NOP   I INS   O OUT   D DEL   C ERR   R RUN
    q     quit
"""

from __future__ import annotations

import socket
import sys
import time

#: char -> (label, row, group). group 1/2/3 = P1.5/6/7; index = row+1+(group-1)*8.
#: Axis-select = group 1 rows 1..6 -> axis 0..5 (rob3-firmware-sim skill).
KEYMAP = {
    "0": ("0", 0, 2), "1": ("1", 1, 2), "2": ("2", 2, 2), "3": ("3", 3, 2),
    "4": ("4", 4, 2), "5": ("5", 5, 2), "6": ("6", 6, 2), "7": ("7", 7, 2),
    "8": ("8", 0, 1), "9": ("9", 1, 1),
    "+": ("UP/RIGHT", 3, 1), "-": ("DOWN/LEFT", 4, 1),
    "P": ("POS", 2, 3), "E": ("ENT", 5, 1), "N": ("NOP", 2, 1),
    "D": ("DEL", 7, 1), "C": ("ERR", 6, 1), "R": ("RUN", 7, 3),
    "I": ("INS", 0, 3), "O": ("OUT", 1, 3),
}


class TeachboxDriver:
    """Drive a ROB3 firmware keypad on a ucSim engine (the single stepper owns
    the engine and calls these). Verified against the ROM:

    * axis-select via the release->hold->release debounce cadence (sets POSITION
      mode 0x29=0x40), per the rob3-firmware-sim skill.
    * jog via the firmware's verified kh_jog (0x0E26) on the selected axis.
    """

    JOG = 0x0E26
    KBD_HANDLE = 0x0C80
    POS_SLOT = 0x50

    def __init__(self, eng):
        self.eng = eng
        self.selected = None

    def _settle(self, n):
        for _ in range(n):
            self.eng.run_cycles(8000)

    def press(self, row, group):
        """Dispatch one keypad key via the verified debounce cadence."""
        if group not in (1, 2, 3):
            return
        if group == 1 and 1 <= row <= 6:          # axis-select
            self.selected = row - 1
        self.eng.release(); self._settle(3)        # release sets flag 0x20.6
        self.eng.press(row, group)
        self.eng.command("break 0x%04x" % self.KBD_HANDLE)
        for _ in range(30):
            out = self.eng.command("step 8000", timeout=15)
            if ("0x%04x" % self.KBD_HANDLE) in out.lower() or \
               ("0x%06x" % self.KBD_HANDLE) in out.lower():
                break
        self.eng.command("clear 0x%04x" % self.KBD_HANDLE)
        self._settle(6)
        self.eng.release(); self._settle(2)

    def axis(self, n):
        """Select axis n (0..5) = group 1, row n+1."""
        self.press(n + 1, 1)

    def jog(self, direction):
        """Jog the selected axis via kh_jog (ACC.0: 0=+ increment, 1=- decrement)."""
        if self.selected is None:
            return
        self.eng.command("set mem sfr 0xd0 0x00")
        self.eng.command("set mem sfr 0xe0 0x%02x" % (direction & 1))
        self.eng.command("set mem iram 0x29 0x40")
        self.eng.command("set mem iram 0x01 0x%02x" % (self.POS_SLOT + self.selected))
        self.eng.command("pc 0x%04x" % self.JOG)
        for bp in ("0x0e40", "0x0e2f", "0x0e36"):
            self.eng.command("break %s" % bp)
        self.eng.run(timeout=10)
        for bp in ("0x0e40", "0x0e2f", "0x0e36"):
            self.eng.command("clear %s" % bp)

    def apply_intent(self, line: str) -> None:
        """Apply one intent line: 'press R G' | 'axis N' | 'jog +|-'."""
        parts = line.split()
        if not parts:
            return
        cmd = parts[0]
        try:
            if cmd == "press" and len(parts) == 3:
                self.press(int(parts[1]), int(parts[2]))
            elif cmd == "axis" and len(parts) == 2:
                self.axis(int(parts[1]))
            elif cmd == "jog" and len(parts) == 2:
                self.jog(0 if parts[1] == "+" else 1)
        except ValueError:
            pass


# --- intent protocol (viewer <- keyboard-teachbox) ---------------------------
def key_to_intent(ch: str) -> str | None:
    """Map a keyboard char to an intent line, or None."""
    k = ch.upper() if ch.upper() in KEYMAP else ch
    if k not in KEYMAP:
        return None
    if k in ("+", "-"):
        return "jog +" if k == "+" else "jog -"
    _, row, group = KEYMAP[k]
    return f"press {row} {group}"


# --- own-engine mode ---------------------------------------------------------
def _own_engine_driver(hex_path=None):
    import os
    from rob3_ucsim import UCSimEngine, default_hex, find_ucsim, MAIN_LOOP
    rom = hex_path or default_hex()
    if not rom:
        raise SystemExit("set ROB3_HEX=/path/to/rom.hex (or pass --hex)")
    base = os.environ.get("ROB3_MODS")
    if not base:
        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        for c in (os.path.join(here, "..", "..", "simulator", "ucsim-modules"),
                  os.path.expanduser("~/github/eurobtec/rob3_ucsim/simulator/ucsim-modules")):
            if os.path.isdir(c):
                base = c
                break
    mods = [os.path.join(base, n, f"{n}.so") for n in ("loopback", "teachbox", "adc")
            if base and os.path.exists(os.path.join(base, n, f"{n}.so"))]
    eng = UCSimEngine(hex_path=rom, binary=find_ucsim(), load_hw=mods or None)
    eng.reset(fixed_baud=False)
    eng.command("set mem sfr 0xb0 0x00")
    if not eng.has_modules:
        print("WARNING: teachbox/adc cl_hw not loaded — keypresses will not reach "
              "the firmware. Build them / set ROB3_MODS.", file=sys.stderr)
    else:
        eng.run_to(MAIN_LOOP)
    return eng, TeachboxDriver(eng)


def _read_key():
    import termios
    import tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def _keyloop(send, *, raw=True):
    """Read keys and call send(intent_line) for each."""
    print("keyboard-teachbox: 0-9 (axis), + - (jog), P E N I O D C R, 'q' quit")
    while True:
        if raw and sys.stdin.isatty():
            ch = _read_key()
        else:
            line = sys.stdin.readline()
            if not line:
                break
            ch = line.strip()[:1]
        if not ch or ch in ("q", "\x03", "\x04"):
            break
        intent = key_to_intent(ch)
        if intent:
            send(intent)
            print(f"  -> {intent}")


def main() -> int:
    import argparse

    ap = argparse.ArgumentParser(description="Keyboard-teachbox: drive the ROB3 firmware keypad")
    ap.add_argument("--intent-port", type=int, default=None,
                    help="attach to a running viewer's intent socket on "
                         "localhost:PORT (opened by `rob3-viz --intent-port PORT`)")
    ap.add_argument("--host", default="localhost", help="intent host (attach mode)")
    ap.add_argument("--hex", default=None, help="ROM image for own-engine mode (default $ROB3_HEX)")
    ap.add_argument("--line", action="store_true", help="line mode (type keys + Enter)")
    args = ap.parse_args()

    if args.intent_port:
        sock = socket.create_connection((args.host, args.intent_port), timeout=5.0)

        def send(intent):
            sock.sendall((intent + "\n").encode())
        try:
            _keyloop(send, raw=not args.line)
        except KeyboardInterrupt:
            pass
        finally:
            sock.close()
    else:
        eng, drv = _own_engine_driver(hex_path=args.hex)
        try:
            _keyloop(lambda intent: drv.apply_intent(intent), raw=not args.line)
        except KeyboardInterrupt:
            pass
        finally:
            eng.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
