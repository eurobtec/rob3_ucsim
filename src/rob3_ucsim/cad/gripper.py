#!/usr/bin/env python3
"""ROB3 gripper (axis 5) - parametric 3D CAD.

Builds the two-finger four-bar (scissor) gripper seen in the bench photos
(hardware/gripper-closed.jpg / -opened.jpg / -top-part.jpg) and exports it in
the SAME formats as the rest of the ROB3 CAD:

  - STL   (mesh for the URDF, like urdf/meshes/*.stl; mm units)
  (This parametric script is the solid source of record; no STEP is written.)

All dimensions are named parameters below. Provenance per value:
  [HW]    = measured on the robot (caliper)   -> trusted
  [INFER] = estimated from the photo 5 mm grid -> PLACEHOLDER, replace with [HW]

Run (from the rob3_ucsim repo, with cadquery in the venv):
    .venv/bin/python <this script>
"""

import math

# cadquery is only needed to BUILD/EXPORT solids; the parameters and the
# analytic geometry helpers import without it (so tests need no CAD kernel).
try:
    import cadquery as cq
except ImportError:  # pragma: no cover
    cq = None

# ---------------------------------------------------------------------------
# Parameters (mm). Edit these as measurements come in.
# ---------------------------------------------------------------------------

# Thicknesses (out-of-plane, the +/- Y direction)        PROVENANCE
PALM_THK      = 15.2   # [HW] palm plate thickness
FINGER_THK    =  9.4   # [HW] finger link thickness
TIP_THK       = 15.2   # [HW] fingertip thickness

# Palm plate (the horizontal cross-bar)                  PROVENANCE
PALM_W        = 75.0   # [HW] width  (left-right)
PALM_H        = 20.0   # [HW] height (top-bottom)

# Wrist mount cylinder: Ø17 x 50 mm, mounted to the palm at its bottom end.
# The axis mount hole (to the previous axis = wrist roll) is 41 mm from the palm
# end / 9 mm from the top end. So the axis sits 41 mm above the palm top.   PROV
HUB_DIA       = 17.0   # [HW] cylinder diameter
HUB_LEN       = 50.0   # [HW] cylinder length (palm end -> top end)
HUB_AXIS_FROM_PALM = 41.0  # [HW] axis mount hole, 41 mm up from the palm (9 from top)
# The wrist axis is therefore this far above the palm top:
AXIS_TO_PALM  = HUB_AXIS_FROM_PALM  # [HW] = 41 mm  (supersedes the earlier 4.2)
HUB_H         = HUB_LEN

# Finger four-bar links: each finger is TWO parallel sticks.      PROVENANCE
LINK_LEN      =  50.0  # [HW] link length (top pivot hole -> bottom tip-mount hole)
STICK_W       =   6.0  # [HW] width of ONE stick
LINK_W        =  14.0  # [HW] total width of the two sticks (outer-to-outer)
# each stick has a hole at BOTH ends: top hole pivots to the palm, bottom hole
# pivots to the tip (makes the four-bar). Pin/hole diameter:
PIN_DIA       =   2.0  # [HW] pivot pin / hole diameter (small, negligible)
# the two sticks of one finger are 8 mm apart (center-to-center): an INNER stick
# (closer to the gripper center) and an OUTER stick. Their top pivots sit at a
# fixed x from the gripper centerline, PIVOT_BELOW_TOP below the palm top.
PIVOT_DX      =   8.0   # [HW] inner<->outer stick center-to-center spacing
INNER_PIVOT_X =  21.5   # [HW] inner stick top-pivot x from center (+/- per side)
OUTER_PIVOT_X =  29.5   # [HW] outer stick top-pivot x from center (+/- per side)
# compatibility shim: nominal pivot-group center + half-spacing picks the sticks
PIVOT_X_FROM_CENTER = (INNER_PIVOT_X + OUTER_PIVOT_X) / 2.0  # = 25.5 mm
# the finger pivot hole sits this far BELOW the top of the palm
PIVOT_BELOW_TOP = 15.0 # [HW] pivot hole depth from palm top
# horizontal gap between the left-finger and right-finger pivot groups
FINGER_GAP    =  52.0  # [HW] gap between the two fingers at the palm

# Fingertips: right triangle with two equal legs (per bench photos).      PROV
#   - inner leg: VERTICAL gripping face
#   - top leg: HORIZONTAL (joins the link); its outer corner is ROUNDED
#   - hypotenuse from the vertical face's bottom up to the outer (rounded) corner
TIP_LEG       =  25.0  # [HW] both equal legs (inner vertical = top horizontal)
TIP_W         = TIP_LEG  # kept for compatibility (base width = top leg)
# The tip's vertical GRIPPING FACE sits this far INBOARD (toward the gripper
# center) of the inner stick's bottom hole.
GRIP_FACE_INBOARD = 7.5  # [HW] gripping face offset inboard of inner bottom hole
# The tip connects to the link via a hole on the TOP (horizontal) leg, this far
# in from the top edge / outer corner:
TIP_HOLE_FROM_TOP = 5.0  # [HW] connection hole, 5 mm from top of the horizontal side
# how far the tip overlaps UP onto the stick ends so they connect (no visual gap)
TIP_OVERLAP   =   6.0  # mm, structural overlap at the stick<->tip joint

# Overall closed length: axis center -> fingertip end, gripper CLOSED (vertical).
AXIS_TO_TIP_CLOSED = 125.0  # [HW]
# Open jaw span: distance between the two fingers when fully OPEN.
OPEN_SPAN     =  85.0  # [HW] tip gap, fingers OPEN
STRAIGHT_SPAN =  28.0  # [HW] tip gap, fingers STRAIGHT (vertical, 0 deg)
# finger pivot distance from the gripper centerline, solved from the straight
# span: each tip inner face at STRAIGHT_SPAN/2, which is pivot_x - TIP_LEG/2.
CLOSED_SPAN   =   0.0  # [HW] tip gap, fingers CLOSED (tips meet)
# Scissor kinematic [HW]: as the gripper OPENS, the fingers rotate about their
# pivots so the 52 mm top gap (FINGER_GAP, closed) CLOSES toward 0 while the
# fingertips spread apart to OPEN_SPAN (85 mm). Closed <-> open are opposite:
#   closed: fingers 52 mm apart at palm, tips together
#   open:   fingers ~0 apart at palm, tips 85 mm apart
# Vertical chain if everything were straight down:
#   AXIS_TO_PALM(4.2) + PIVOT_BELOW_TOP(15) + LINK_LEN(50) + TIP_LEG(25) = 94.2
# ...but the measured closed length is 125 mm. The 30.8 mm difference means the
# links/tip are NOT purely vertical when closed (the four-bar hangs at an angle),
# OR the tip extends past its leg. Tracked as a reconciliation note; the vertical
# render below uses the measured leg lengths (tip is geometric, not stretched).
TIP_VSPAN     = TIP_LEG  # [HW] tip vertical extent = inner leg (25 mm)

# Scissor poses (splay of the fingers about their palm pivots).       PROVENANCE
#   Parallelogram four-bar (the tip stays VERTICAL and only translates). theta is
#   measured from vertical; solved + cross-validated against the measured stick-
#   hole spacings and tip spans:
#     CLOSED  theta = -14.5 deg -> inner holes +/-9mm  (18 apart),  tip span ~0
#     STRAIGHT theta = 0 deg    -> inner holes +/-21.5mm,           tip span 28mm
#     OPEN    theta = +34.8 deg -> inner holes +/-50mm (100 apart), tip span 85mm
SPLAY_CLOSED_DEG = -14.5  # [HW]-solved: tips meet (span ~0)
SPLAY_STRAIGHT_DEG = 0.0  # [HW]-solved: tips 28 mm apart
SPLAY_OPEN_DEG   =  34.8  # [HW]-solved: tips 85 mm apart
# Default exported pose: OPEN.
FINGER_SPLAY_DEG = SPLAY_OPEN_DEG

# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
# Coordinate convention (matches URDF link frame intent):
#   X = left-right (palm width), Y = thickness, Z = up-down (fingers hang -Z)
# Palm centered at origin, top face at z=0, fingers extend -Z.
import math


def _box(w, d, h):
    return cq.Workplane("XY").box(w, d, h, centered=(True, True, True))


def build(splay_deg):
    """Build the whole gripper solid with the fingers splayed by splay_deg."""
    # Palm plate: top face at z=0, extends downward.
    palm = _box(PALM_W, PALM_THK, PALM_H).translate((0, 0, -PALM_H / 2.0))

    # Wrist cylinder on top of the palm (axis along Z), sits above z=0.
    hub = cq.Workplane("XY").circle(HUB_DIA / 2.0).extrude(HUB_H)

    model = palm.union(hub)

    def finger(sign):
        """sign=-1 left, +1 right. Returns a solid.

        PARALLELOGRAM four-bar. Each finger has TWO parallel sticks (an INNER
        stick at +/-INNER_PIVOT_X and an OUTER stick at +/-OUTER_PIVOT_X) that
        both swing by the SAME angle theta about their own top pivot. Because
        the two sticks stay parallel, the fingertip they carry keeps a FIXED
        (vertical) orientation and simply TRANSLATES along the arc swept by the
        stick bottom-ends.

        theta is measured from vertical; the gripper OPENS (tips track outward)
        as theta grows. CadQuery's ``rotate`` takes two POINTS defining the
        rotation axis, so the pivot axis is (px,0,z0)->(px,1,z0) (parallel to Y
        through the pivot). Empirically (see the marker-box probe below) a
        rotation of ``-sign*theta`` about that axis moves the right finger (+x)
        OUTWARD for +theta, matching the solved hole positions.
        """
        theta = math.radians(splay_deg)
        z0 = -PIVOT_BELOW_TOP                        # pivot height (z)

        def swung_bottom(pivot_x):
            """Return (x, z) of a stick's bottom hole after swinging by theta.

            Determined EMPIRICALLY with a tiny marker box transformed by exactly
            the same rotation the stick receives, so the placement never relies
            on a hand-derived rotation sign.
            """
            rot_deg = -sign * math.degrees(theta)
            m = (cq.Workplane("XY").box(0.001, 0.001, 0.001)
                 .translate((pivot_x, 0, z0 - LINK_LEN))
                 .rotate((pivot_x, 0, z0), (pivot_x, 1, z0), rot_deg))
            c = m.val().Center()
            return c.x, c.z

        # --- the two sticks: build hanging -Z, drill both pin holes, place at
        #     the pivot x, then swing about the Y-parallel pivot axis ---
        sticks = []
        for pivot_x in (sign * INNER_PIVOT_X, sign * OUTER_PIVOT_X):
            s = (cq.Workplane("XY")
                 .box(STICK_W, FINGER_THK, LINK_LEN, centered=(True, True, False)))
            s = s.rotate((0, 0, 0), (1, 0, 0), 180)        # hang -Z
            for hz in (0.0, -LINK_LEN):                     # top + bottom pin holes
                s = s.cut(cq.Workplane("XZ").moveTo(0, hz)
                          .circle(PIN_DIA / 2.0).extrude(FINGER_THK * 2, both=True))
            s = s.translate((pivot_x, 0, z0))              # to its pivot
            rot_deg = -sign * math.degrees(theta)
            s = s.rotate((pivot_x, 0, z0), (pivot_x, 1, z0), rot_deg)  # swing (arc)
            sticks.append(s)

        # Inner stick bottom-hole position after the swing (anchors the tip).
        inner_hole_x, inner_hole_z = swung_bottom(sign * INNER_PIVOT_X)

        # --- fingertip: VERTICAL right triangle (never rotated), gripping face
        #     INWARD. The vertical gripping face sits GRIP_FACE_INBOARD inboard
        #     (toward the center) of the inner stick's bottom hole. Top leg is
        #     HORIZONTAL, outer corner rounded. The tip overlaps up onto the
        #     stick ends (no gap). ---
        xface = inner_hole_x - sign * GRIP_FACE_INBOARD     # vertical gripping face
        xout  = xface + sign * TIP_LEG                       # outer corner (top leg)
        topz  = inner_hole_z + TIP_OVERLAP                   # overlap up onto stick
        A = (xface, topz)
        B = (xout, topz)
        C = (xface, topz - TIP_LEG)
        tip = cq.Workplane("XZ").moveTo(*A).lineTo(*B).lineTo(*C).close().extrude(TIP_THK)
        try:
            tip = tip.edges("|Y").fillet(min(8.0, TIP_LEG / 3.0))
        except Exception:
            pass
        tip = tip.translate((0, -TIP_THK / 2.0, 0))

        finger_solid = sticks[0].union(sticks[1]).union(tip)
        return finger_solid

    return model.union(finger(-1)).union(finger(+1))


# ---------------------------------------------------------------------------
# Analytic geometry helpers (no CAD kernel needed) — shared by the tests so the
# model math and the asserted dimensions stay in sync.
# ---------------------------------------------------------------------------
def finger_center_x():
    """X of a finger's pivot group center from the gripper centerline (mm)."""
    return PIVOT_X_FROM_CENTER


def inner_hole_x(splay_deg):
    """X of the inner stick's bottom hole for the right finger (+x side), mm.

    Parallelogram four-bar: the bottom hole = inner pivot + LINK_LEN*sin(theta),
    with theta measured from vertical. Matches the empirical CadQuery placement
    (right finger moves OUTWARD for +theta)."""
    return INNER_PIVOT_X + math.sin(math.radians(splay_deg)) * LINK_LEN


def open_inner_span(splay_deg):
    """Inner-face (gripping-face) separation between the two fingers (mm).

    The vertical gripping face sits GRIP_FACE_INBOARD inboard of the inner
    bottom hole, so the full tip-to-tip span is twice that face offset."""
    face = inner_hole_x(splay_deg) - GRIP_FACE_INBOARD
    return 2.0 * face


def axis_to_tip(splay_deg):
    """Axis-center → fingertip-end vertical length (mm) at the given splay.

    Parallelogram: the tip hangs VERTICAL from the inner stick's swung bottom
    hole, so the fingertip bottom is TIP_LEG below that hole's z. The hole z is
    the inner pivot swung by theta: z0 - LINK_LEN*cos(theta). The wrist axis is
    AXIS_TO_PALM above the palm top (z=0)."""
    hole_z = -PIVOT_BELOW_TOP - math.cos(math.radians(splay_deg)) * LINK_LEN
    tip_bottom = hole_z + TIP_OVERLAP - TIP_LEG   # tip overlaps up, extends down
    return AXIS_TO_PALM - tip_bottom


# ---------------------------------------------------------------------------
# Export (STL only, mm). Default = OPEN; also emit named open/closed poses.
# The STL is what the URDF/sim loads; the solid source of record is this
# parametric script, so no STEP is written (regenerate any time by re-running).
# ---------------------------------------------------------------------------
import os

HERE = os.path.dirname(os.path.abspath(__file__))


def _export(model, name):
    stl_out = os.path.join(HERE, f"{name}.stl")
    cq.exporters.export(model, stl_out, tolerance=0.1, angularTolerance=0.1)
    print("wrote:", stl_out)


if __name__ == "__main__":
    # default gripper.* = OPEN pose
    _export(build(SPLAY_OPEN_DEG), "gripper")
    # named reference poses
    _export(build(SPLAY_OPEN_DEG), "gripper_open")
    _export(build(SPLAY_CLOSED_DEG), "gripper_closed")
