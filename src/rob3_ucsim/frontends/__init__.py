"""ROB3 visualization frontends (viewers) + the live-follow run loop.

Viewers are read-only renderers of the firmware's axis positions. Pick one with
``--backend``:

* ``text``     — terminal: per-axis current value vs min/max (ASCII bars). Default.
* ``pybullet`` — 3D URDF window with a native orbit camera (needs ``[viz]`` extra).

The firmware is driven SEPARATELY (teachbox / serial / stored program). To drive
it from a keyboard while watching here, run the viewer with ``--console-port N``
and attach a keyboard-teachbox over that console socket in another terminal.
"""

from __future__ import annotations

import time

from ..axes import DEFAULT_INTENT_PORT
from .text_view import TextViewer

#: name -> factory. pybullet is imported lazily so the package works without the
#: optional dependency.
_REGISTRY = {
    "text": lambda gui=True: TextViewer(),
}


def available() -> list[str]:
    names = list(_REGISTRY) + ["pybullet"]
    return sorted(set(names))


def make_viewer(backend: str, *, gui: bool = True):
    """Construct a viewer by name (``text`` / ``pybullet``)."""
    if backend == "pybullet":
        from .pybullet_view import PyBulletViewer
        return PyBulletViewer(gui=gui)
    try:
        return _REGISTRY[backend](gui=gui)
    except KeyError:
        raise SystemExit(f"unknown --backend {backend!r}; choices: {', '.join(available())}")


def _default_modules() -> list[str]:
    """Locate the adc + loopback cl_hw .so modules (so the firmware reaches the
    main loop and the servo actually moves the axes). Overridable via ROB3_MODS.
    Returns the ones that exist; empty if none found (viewer then shows a static
    pose and warns)."""
    import os
    base = os.environ.get("ROB3_MODS")
    if not base:
        # Shipped alongside this package's simulator tree, or a sibling checkout.
        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        for cand in (
            os.path.join(here, "..", "..", "simulator", "ucsim-modules"),
            os.path.expanduser("~/github/eurobtec/rob3_ucsim/simulator/ucsim-modules"),
        ):
            if os.path.isdir(cand):
                base = cand
                break
    mods = []
    if base:
        # adc + loopback: firmware reaches its servo loop. teachbox: so a
        # keyboard-teachbox attached over --console-port can drive the keypad on
        # this (shared) ucSim.
        for name in ("adc/adc.so", "loopback/loopback.so", "teachbox/teachbox.so"):
            p = os.path.join(base, name)
            if os.path.exists(p):
                mods.append(p)
    return mods


def run(backend: str = "text", *, hex_path: str | None = None, hz: float = 20.0,
        cycles_per_frame: int = 20000, gui: bool = True,
        console_port: int | None = None) -> None:
    """Boot the ROB3 ROM in ucSim, free-run it, and render live via ``backend``.

    Needs the ROM (``ROB3_HEX`` or ``hex_path``) and a ucSim binary (``UCSIM_51``).
    Loads the adc + loopback cl_hw modules (``ROB3_MODS`` to override) so the
    firmware reaches its main loop and the servo moves the axes — without them
    the arm renders but never moves. If ``console_port`` is set, a separate
    keyboard-teachbox can attach to the same ucSim over that socket to drive the
    firmware while this viewer renders.
    """
    from .. import UCSimEngine, default_hex, find_ucsim, MAIN_LOOP

def _start_intent_server(port: int, q):
    """Accept intent-line connections on localhost:port; push lines into queue q.

    The viewer (sole ucSim owner) runs this; input drivers (keyboard-teachbox,
    future host-RS232) connect and send intent lines. One stepper, many intent
    producers — see the rob3-firmware-sim skill.
    """
    import socket
    import threading

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port))
    srv.listen(4)

    def serve():
        while True:
            try:
                conn, _ = srv.accept()
            except OSError:
                return
            threading.Thread(target=_client, args=(conn,), daemon=True).start()

    def _client(conn):
        buf = b""
        with conn:
            while True:
                try:
                    data = conn.recv(256)
                except OSError:
                    return
                if not data:
                    return
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    q.put(line.decode(errors="ignore").strip())

    threading.Thread(target=serve, daemon=True).start()
    return srv


def run(backend: str = "text", *, hex_path: str | None = None, hz: float = 20.0,
        cycles_per_frame: int = 20000, gui: bool = True,
        intent_port: int | None = None) -> None:
    """Boot the ROB3 ROM in ucSim, free-run it, and render live via ``backend``.

    Needs the ROM (``ROB3_HEX`` or ``hex_path``) and a ucSim binary (``UCSIM_51``).
    Loads the adc + loopback + teachbox cl_hw modules (``ROB3_MODS`` to override)
    so the firmware runs its servo and the keypad is drivable.

    This process is the SOLE ucSim owner + stepper. If ``intent_port`` is set, it
    opens an intent socket: input drivers (``rob3-teachbox``, or a
    future host-RS232 driver) connect and send intent lines (``press R G`` /
    ``axis N`` / ``jog +``); the loop applies each via the verified cadence
    between frames, then renders. (pyucsim is single-owner — never add a second
    engine/console. See the rob3-firmware-sim skill.)
    """
    import queue
    import sys
    import os

    from .. import UCSimEngine, default_hex, find_ucsim, MAIN_LOOP
    from ..inputs.teachbox import TeachboxDriver

    rom = hex_path or default_hex()
    if not rom:
        raise SystemExit("set ROB3_HEX=/path/to/rom.hex (or pass --hex)")

    mods = _default_modules()
    viewer = make_viewer(backend, gui=gui)
    eng = UCSimEngine(hex_path=rom, binary=find_ucsim(), load_hw=mods or None)
    intents: "queue.Queue[str]" = queue.Queue()
    srv = None
    try:
        eng.reset(fixed_baud=False)
        eng.command("set mem sfr 0xb0 0x00")   # de-assert emergency-off (P3.2)
        # Startup status so you can see WHAT loaded and whether it is live.
        print("ROB3 ucSim viewer", file=sys.stderr)
        print(f"  ucSim  : {getattr(eng, 'binary', '?')}", file=sys.stderr)
        print(f"  ROM    : {rom}", file=sys.stderr)
        modnames = ", ".join(os.path.basename(m) for m in mods) or "(none)"
        print(f"  cl_hw  : {modnames}", file=sys.stderr)
        print(f"  servo  : {'LIVE (modules loaded)' if eng.has_modules else 'STATIC — set ROB3_MODS to move'}",
              file=sys.stderr)
        if not eng.has_modules:
            print("WARNING: cl_hw modules did not load — the firmware servo will "
                  "not run and the arm will not move. Most likely the ucSim binary "
                  "can't load plugins: point UCSIM_51 at the loader-enabled "
                  "ucsim_51 (not stock s51). Modules tried: " + (modnames) +
                  (". None found — set ROB3_MODS." if not mods else "."),
                  file=sys.stderr)
        else:
            eng.run_to(MAIN_LOOP)
        drv = TeachboxDriver(eng)
        if intent_port:
            srv = _start_intent_server(intent_port, intents)
            print(f"intent socket: localhost:{intent_port}  "
                  f"(attach: rob3-teachbox)",
                  file=sys.stderr)
        period = 1.0 / hz
        last_intent = ""
        while True:
            # Apply any queued input intents using the verified cadence (we are
            # the single stepper), then advance the servo and render.
            while not intents.empty():
                last_intent = intents.get_nowait()
                drv.apply_intent(last_intent)
            eng.run_cycles(cycles_per_frame)
            try:
                viewer.set_positions(eng.read_positions(), status=drv.last_action)
            except TypeError:
                viewer.set_positions(eng.read_positions())   # viewers w/o status
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        if srv is not None:
            srv.close()
        eng.close()
        viewer.close()


def main() -> int:
    import argparse

    ap = argparse.ArgumentParser(
        description="Visualize live ROB3 movement (viewer frontends)")
    ap.add_argument("--backend", default="text",
                    help="viewer: text (default) | pybullet")
    ap.add_argument("--hex", default=None, help="ROM image (default: $ROB3_HEX)")
    ap.add_argument("--hz", type=float, default=20.0, help="frame rate (default 20)")
    ap.add_argument("--cycles", type=int, default=20000,
                    help="ucSim cycles advanced per frame (default 20000)")
    ap.add_argument("--port", type=int, default=None,
                    help="intent socket port for input drivers (default "
                         f"{DEFAULT_INTENT_PORT}; 0 to disable). rob3-teachbox "
                         "connects here to drive the firmware while this renders")
    ap.add_argument("--headless", action="store_true",
                    help="pybullet DIRECT mode (no window)")
    args = ap.parse_args()
    port = DEFAULT_INTENT_PORT if args.port is None else args.port
    run(args.backend, hex_path=args.hex, hz=args.hz, cycles_per_frame=args.cycles,
        gui=not args.headless, intent_port=(port or None))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
