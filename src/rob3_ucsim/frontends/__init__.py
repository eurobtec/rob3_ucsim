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
        for name in ("adc/adc.so", "loopback/loopback.so"):
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

    rom = hex_path or default_hex()
    if not rom:
        raise SystemExit("set ROB3_HEX=/path/to/rom.hex (or pass --hex)")

    mods = _default_modules()
    viewer = make_viewer(backend, gui=gui)
    eng = UCSimEngine(hex_path=rom, binary=find_ucsim(), console_port=console_port,
                      load_hw=mods or None)
    try:
        eng.reset(fixed_baud=False)
        if not eng.has_modules:
            import sys
            print("WARNING: adc/loopback cl_hw modules not loaded — the firmware "
                  "will not run its servo and the arm will not move. Build them "
                  "(simulator/ucsim-modules) or set ROB3_MODS.", file=sys.stderr)
        # Free-run drives the ADC servo loop; read_positions() reflects the live
        # pot state. Motion appears when something drives the firmware (a
        # keyboard-teachbox over --console-port, serial, or a stored program).
        period = 1.0 / hz
        while True:
            eng.run_cycles(cycles_per_frame)
            viewer.set_positions(eng.read_positions())
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
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
    ap.add_argument("--console-port", type=int, default=None,
                    help="expose the ucSim console on localhost:PORT so a "
                         "keyboard-teachbox can attach and drive the firmware")
    ap.add_argument("--headless", action="store_true",
                    help="pybullet DIRECT mode (no window)")
    args = ap.parse_args()
    run(args.backend, hex_path=args.hex, hz=args.hz, cycles_per_frame=args.cycles,
        gui=not args.headless, console_port=args.console_port)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
