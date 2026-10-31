"""Turbofan engine v5 — airfoil blades, stator rows, pylon.

Changes over v4, following the reference CAD stack:
  * every blade/vane is a cambered NACA-4-digit airfoil lofted root -> tip with
    twist (stagger grows along the span) instead of a rectangular plate;
  * each compressor stage now has a rotor row AND a stator row (vanes hung from
    the casing); the turbine keeps 4 stage pairs;
  * a swept pylon on the -X side, which becomes "up" once the engine is laid
    horizontally (the whole assembly is rotated 90 deg about Y at the end).

Axis before the final rotation: +Z (0 = intake lip). Millimetres. ~3.0 m long.
"""

import math

import build123d as bd
from cadgen import model
from cadgen.ap242 import configuration_sequence, mechanism_state

CENTER = bd.Align.CENTER
MIN = bd.Align.MIN
ALIGN_Z = (CENTER, CENTER, MIN)

FAN_TIP = 900.0
NACELLE_OUTER = 980.0
CORE_CASE = 480.0
SHAFT_R = 90.0
CLEAR = 1.2          # rotor tip / stator tip clearance, mm
COMP_PITCH = 112.0   # axial pitch between compressor stages, mm
TURB_PITCH = 120.0   # axial pitch between turbine stages, mm

POLISHED = "#dfe3e8"
TITANIUM = "#b9bfc8"
BLADE = "#cfd4db"
DARK = "#3f444b"
HOT = "#9a7458"
WHITE = "#f0f2f5"
CORE = "#7c838d"

# which parts turn with which spool (label prefixes)
LP_PREFIXES = ("spinner", "fan_hub", "fan_blade", "boost_rotor", "booster_drum",
               "turb_rotor_3", "turb_rotor_4", "turb_disk_3", "turb_disk_4")
HP_PREFIXES = ("comp_rotor", "comp_disk", "compressor_drum",
               "turb_rotor_1", "turb_rotor_2", "turb_disk_1", "turb_disk_2", "shaft")


# --------------------------------------------------------------------------- #
# geometry helpers
# --------------------------------------------------------------------------- #
def cyl(radius, z0, height, align=ALIGN_Z):
    return bd.Cylinder(radius, height, align=align).moved(bd.Location((0, 0, z0)))


def frustum(r0, r1, z0, height):
    if abs(r0 - r1) < 1e-9:
        return cyl(r0, z0, height)
    return bd.Cone(r0, r1, height, align=ALIGN_Z).moved(bd.Location((0, 0, z0)))


def ring(r0, r1, ri0, ri1, z0, height):
    return frustum(r0, r1, z0, height) - frustum(ri0, ri1, z0 - 2, height + 4)


def airfoil(n=14, camber=0.05, thickness=0.08, chord=1.0, stagger_deg=0.0):
    """Closed cambered outline in (u, v), rotated by the stagger angle."""
    upper, lower = [], []
    for i in range(n + 1):
        x = 0.5 * (1 - math.cos(math.pi * i / n))
        yt = 5 * thickness * (0.2969 * math.sqrt(x) - 0.1260 * x - 0.3516 * x ** 2
                              + 0.2843 * x ** 3 - 0.1015 * x ** 4)
        yc = camber * 4 * x * (1 - x)
        upper.append(((x - 0.5) * chord, (yc + yt) * chord))
        lower.append(((x - 0.5) * chord, (yc - yt) * chord))
    pts = upper + lower[::-1][1:-1]
    c, s = math.cos(math.radians(stagger_deg)), math.sin(math.radians(stagger_deg))
    return [(u * c - v * s, u * s + v * c) for u, v in pts]


def blade_face(radius, z, chord, thickness, camber, stagger_deg):
    pts = airfoil(camber=camber, thickness=thickness, chord=chord, stagger_deg=stagger_deg)
    plane = bd.Plane(origin=(radius, 0, z), x_dir=(0, 0, 1), z_dir=(1, 0, 0))
    return plane * bd.Polygon(*pts, align=None)


def vane(radius_root, radius_tip, z_root, z_tip, chord_root, chord_tip,
         thick_root, thick_tip, camber, stagger_root, stagger_tip):
    """One airfoil blade: lofted root -> tip with twist and taper."""
    root = blade_face(radius_root, z_root, chord_root, thick_root, camber, stagger_root)
    tip = blade_face(radius_tip, z_tip, chord_tip, thick_tip, camber * 0.7, stagger_tip)
    return bd.loft([root, tip], ruled=True)


def row(count, *args, color=BLADE, label="blade", phase=0.0):
    parts = []
    one = vane(*args)
    for i in range(count):
        piece = one.rotate(bd.Axis.Z, phase + i * 360.0 / count)
        piece.color = color
        piece.label = f"{label}_{i}"
        parts.append(piece)
    return parts


def radial_boss(radius, z, boss_r, boss_h, count, color=DARK, label="boss"):
    parts = []
    one = bd.Cylinder(boss_r, boss_h, align=(CENTER, CENTER, MIN)).rotate(bd.Axis.Y, 90)
    one = one.moved(bd.Location((radius, 0, z)))
    for i in range(count):
        piece = one.rotate(bd.Axis.Z, i * 360.0 / count)
        piece.color = color
        piece.label = f"{label}_{i}"
        parts.append(piece)
    return parts


def _tag(shape, label, color):
    shape.label = label
    shape.color = color
    return shape


# --------------------------------------------------------------------------- #
# the engine
# --------------------------------------------------------------------------- #
@model
def turbofan():
    parts = []

    # ---------------- intake + fan ----------------
    parts.append(_tag(frustum(285, 0, 170, 300), "spinner", POLISHED))
    parts.append(_tag(frustum(300, 320, 440, 150), "fan_hub", TITANIUM))

    parts += row(20, 320, 901, 470, 556, 210, 150, 0.10, 0.055, 0.055, 30, 62,
                 color=BLADE, label="fan_blade")

    parts.append(_tag(bd.Torus(950, 34).moved(bd.Location((0, 0, 405))), "intake_lip", WHITE))
    parts.append(_tag(ring(NACELLE_OUTER, 930, 905, 862, 420, 1080), "nacelle", WHITE))

    # ---------------- core cowl + casing ----------------
    parts.append(_tag(ring(560, 480, 520, 440, 940, 1260), "core_cowl", CORE))

    # ---------------- bypass OGV: large vanes at the fan exit ------------
    parts += row(36, 556, 882, 980, 984, 190, 150, 0.10, 0.07, 0.05, 6, 12,
                 color=POLISHED, label="ogv", phase=2.5)
    parts.append(_tag(ring(450, 430, 445, 392, 560, 1850), "core_casing", "#5c636c"))

    # ---------------- LP booster: 3 stages between fan and HP compressor ----
    parts.append(_tag(cyl(300, 580, 190), "booster_drum", TITANIUM))
    boost = [
        # pitch, drum radius, casing inner radius, rotors, stators, stagger, chord
        (68.0, 300.0, 443.8, 56, 60, 12, 24),
        (64.0, 300.0, 441.9, 60, 64, 13, 22),
        (60.0, 300.0, 440.1, 64, 68, 14, 20),
    ]
    z = 600.0
    for i, (pitch, drum_r, case_r, n_rot, n_sta, stagger, chord) in enumerate(boost, 1):
        parts += row(n_rot, drum_r, case_r - CLEAR, z, z + 4, chord, chord * 0.80,
                     0.10, 0.060, 0.05, stagger, stagger + 12,
                     color=BLADE, label=f"boost_rotor_{i}")
        parts += row(n_sta, case_r, drum_r + CLEAR, z + pitch / 2, z + pitch / 2 + 2,
                     chord * 0.85, chord * 0.70, 0.080, 0.070, 0.03, 3, 6,
                     color=POLISHED, label=f"boost_stator_{i}", phase=4.0)
        z += pitch

    # ---------------- compressor: drum, 7 rotor rows + 7 stator rows -----
    parts.append(_tag(frustum(300, 250, 760, 790), "compressor_drum", TITANIUM))

    # pitch is per stage: pitch = 2 * (AXIAL_GAP + half the rotor+stator axial extent),
    # so every rotor/stator pair keeps the same 12 mm gap on both sides.
    comp = [
        # pitch, drum radius, casing inner radius, rotors, stators, stagger, chord
        (121.0, 297.5, 438.0, 36, 40, 20, 54),
        (117.2, 290.5, 435.0, 40, 44, 21, 52),
        (113.4, 283.4, 432.0, 44, 48, 22, 50),
        (109.4, 276.4, 429.0, 48, 52, 23, 48),
        (105.6, 269.4, 425.0, 52, 56, 24, 46),
        (101.6, 262.3, 422.0, 56, 60, 25, 44),
        (97.8, 255.3, 419.0, 60, 64, 26, 42),
    ]
    z = 800.0
    for i, (pitch, drum_r, case_r, n_rot, n_sta, stagger, chord) in enumerate(comp, 1):
        parts.append(_tag(cyl(drum_r + 10, z - 16, 42), f"comp_disk_{i}", TITANIUM))
        parts += row(n_rot, drum_r, case_r - CLEAR, z, z + 5, chord, chord * 0.78,
                     0.10, 0.055, 0.05, stagger, stagger + 14,
                     color=BLADE, label=f"comp_rotor_{i}")
        parts += row(n_sta, case_r, drum_r + CLEAR, z + pitch / 2, z + pitch / 2 + 2,
                     chord * 0.86, chord * 0.70, 0.075, 0.065, 0.03, 4, 8,
                     color=POLISHED, label=f"comp_stator_{i}", phase=6.0)
        z += pitch

    # ---------------- annular combustor ----------------
    parts.append(_tag(ring(478, 478, 430, 430, 1573, 230), "combustor_case", DARK))
    parts.append(_tag(ring(392, 392, 300, 300, 1583, 210), "combustor_liner", HOT))
    parts += radial_boss(470, 1683, 34, 66, 16, color=DARK, label="fuel_boss")
    parts.append(_tag(bd.Torus(520, 13).moved(bd.Location((0, 0, 1773))), "fuel_manifold", "#8b9199"))

    # ---------------- turbine: 4 stage pairs ----------------
    turb = [
        # pitch, disk radius, casing inner radius, rotors, stators, stagger, chord
        (113.8, 340, 407.0, 46, 50, 38, 56),
        (107.6, 332, 403.0, 52, 56, 41, 53),
        (101.2, 324, 400.0, 58, 62, 44, 50),
        (94.8, 316, 396.0, 64, 68, 47, 47),
    ]
    z = 1873.0
    for i, (pitch, disk_r, case_r, n_rot, n_sta, stagger, chord) in enumerate(turb, 1):
        parts.append(_tag(cyl(disk_r, z - 20, 52), f"turb_disk_{i}", HOT))
        parts += row(n_rot, disk_r, case_r - CLEAR, z, z + 4, chord, chord * 0.72,
                     0.09, 0.065, 0.06, stagger, stagger + 10,
                     color="#c79a72", label=f"turb_rotor_{i}")
        parts += row(n_sta, case_r, disk_r + CLEAR, z + pitch / 2, z + pitch / 2 + 2,
                     chord * 0.82, chord * 0.68, 0.08, 0.07, 0.04, 6, 10,
                     color=POLISHED, label=f"turb_stator_{i}", phase=5.0)
        z += pitch

    # ---------------- exhaust ----------------
    parts.append(_tag(ring(470, 370, 425, 330, 2340, 340), "exhaust_nozzle", HOT))
    parts.append(_tag(frustum(370, 65, 2670, 420), "tail_cone", HOT))

    # ---------------- shaft ----------------
    parts.append(_tag(cyl(SHAFT_R, 300, 2480), "shaft", "#767d87"))

    # ---------------- pylon (on -X; becomes "up" after the final rotation) ----
    root_sec = bd.Plane(origin=(-720, 0, 972), x_dir=(1, 0, 0), z_dir=(0, 0, 1)) * \
        bd.Rectangle(820, 78, align=None)
    tip_sec = bd.Plane(origin=(-790, 0, 1350), x_dir=(1, 0, 0), z_dir=(0, 0, 1)) * \
        bd.Rectangle(430, 44, align=None)
    pylon = bd.loft([root_sec, tip_sec], ruled=True)
    parts.append(_tag(pylon, "pylon", CORE))

    # ---------------- quarter cutaway (+X/+Y quadrant removed) ----------
    cutter = bd.Box(2600, 2600, 4000, align=(MIN, MIN, MIN)).moved(bd.Location((0, 0, -400)))
    sectioned = [p.rotate(bd.Axis.Y, 90) - cutter.rotate(bd.Axis.Y, 90) for p in parts]

    # ---------------- mechanism: fixed cases + two spools ----------------
    lp, hp, static = [], [], []
    for p in sectioned:
        name = p.label or ""
        if any(name.startswith(pre) for pre in LP_PREFIXES):
            lp.append(p)
        elif any(name.startswith(pre) for pre in HP_PREFIXES):
            hp.append(p)
        else:
            static.append(p)

    base = bd.Compound(children=static, label="static")
    rotor_lp = bd.Compound(children=lp, label="rotor_lp")
    rotor_hp = bd.Compound(children=hp, label="rotor_hp")

    axis = bd.Axis((0, 0, 0), (1, 0, 0))
    seat = bd.Plane(origin=(0, 0, 0), z_dir=(1, 0, 0)).location
    bd.RevoluteJoint("n1", base, axis=axis, angular_range=(-720, 720))
    bd.RigidJoint("n1", rotor_lp, seat)
    bd.RevoluteJoint("n2", base, axis=axis, angular_range=(-1440, 1440))
    bd.RigidJoint("n2", rotor_hp, seat)
    base.joints["n1"].connect_to(rotor_lp.joints["n1"], angle=0)
    base.joints["n2"].connect_to(rotor_hp.joints["n2"], angle=0)

    assembly = bd.Compound(children=[base, rotor_lp, rotor_hp], label="turbofan")
    # 40 keyframes over one full turn (9 deg per frame): enough for a video loop,
    # and 9 deg is coprime with every blade pitch so no row strobes.
    frames = [(f"p{i:02d}", i * 9.0, i * 18.0) for i in range(40)]
    for name, a1, a2 in frames:
        mechanism_state(assembly, name, n1=a1, n2=a2)
    return configuration_sequence(
        assembly, "run", [(i * 0.075, name) for i, (name, _a1, _a2) in enumerate(frames)] + [(3.0, "p00")]
    )


if __name__ == "__main__":
    turbofan()
