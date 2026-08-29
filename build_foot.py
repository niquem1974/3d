import numpy as np
import trimesh
from trimesh.creation import extrude_polygon
from shapely.geometry import Polygon
import math

# ---------------- FOOT / SOCKET PARAMETERS ----------------
FOOT_DIA        = 36.0   # measured rubber foot diameter, mm
FOOT_CLEARANCE  = 0.4    # total clearance added to bore diameter, mm (tune after a test fit)
SOCKET_DEPTH    = 13.0   # how deep the rubber foot inserts into the printed part, mm
REAR_FOOT_HT    = 15.0   # existing rear foot height, unchanged, mm
CAB_DEPTH       = 390.0  # front-to-back foot spacing measured off the cab, mm
TILT_DEG        = 6.0    # desired tilt back angle, degrees - EDIT THIS TO TRY OTHER ANGLES
WALL            = 7.0    # minimum wall thickness around the socket, mm
FOOTPRINT_W     = 50.0   # left-right footprint width, mm
FOOTPRINT_D     = 65.0   # front-back footprint depth, mm
CHAMFER         = 8.0    # corner chamfer size on the footprint, mm
RIM_CHAMFER     = 1.5    # lead-in chamfer at the socket mouth, mm at 45 degrees

# ---------------- STORAGE CLIP PARAMETERS ----------------
ADD_CLIP        = True   # set False to print the foot on its own, no clip
HANDLE_DIA      = 30.0   # measured cab handle diameter, mm
HANDLE_CLEARANCE= 0.6    # total clearance added to the clip bore, mm
CLIP_WALL       = 3.5    # clip ring wall thickness, mm
CLIP_BAND       = 18.0   # clip width along the handle's length, mm
CLIP_GAP_DEG    = 60.0   # open angle of the C, degrees (snap-fit opening)
CLIP_OVERLAP    = 4.0    # how far the clip ring is pushed into the foot body for a solid weld, mm
CLIP_Z          = 22.0   # height up the foot where the clip is centred, mm

# ---------------- DERIVED VALUES ----------------
rise = CAB_DEPTH * math.tan(math.radians(TILT_DEG))
total_height = REAR_FOOT_HT + rise
bore_dia = FOOT_DIA + FOOT_CLEARANCE
clip_inner_r = (HANDLE_DIA + HANDLE_CLEARANCE) / 2
clip_outer_r = clip_inner_r + CLIP_WALL

print(f"Tilt angle: {TILT_DEG} deg")
print(f"Required rise over {CAB_DEPTH} mm depth: {rise:.1f} mm")
print(f"Total printed foot height: {total_height:.1f} mm")
print(f"Socket bore diameter: {bore_dia:.1f} mm")
print(f"Solid floor thickness below socket: {total_height - SOCKET_DEPTH:.1f} mm")
print(f"Clip inner bore diameter: {clip_inner_r*2:.1f} mm, opening gap: {CLIP_GAP_DEG} deg")

# ---------------- OUTER PRISM (chamfered rectangle footprint) ----------------
hw, hd, c = FOOTPRINT_W / 2, FOOTPRINT_D / 2, CHAMFER
pts = [
    (-hw + c, -hd), (hw - c, -hd), (hw, -hd + c),
    (hw, hd - c), (hw - c, hd), (-hw + c, hd),
    (-hw, hd - c), (-hw, -hd + c),
]
poly = Polygon(pts)
outer = extrude_polygon(poly, height=total_height)

# ---------------- SOCKET BORE ----------------
socket = trimesh.creation.cylinder(radius=bore_dia / 2, height=SOCKET_DEPTH + 1)
socket.apply_translation((0, 0, total_height - SOCKET_DEPTH / 2 + 0.5))

chamfer_cone = trimesh.creation.cone(radius=bore_dia / 2 + RIM_CHAMFER, height=RIM_CHAMFER)
chamfer_cone.apply_translation((0, 0, total_height - RIM_CHAMFER))

foot = outer.difference(socket, engine="manifold")
foot = foot.difference(chamfer_cone, engine="manifold")

# ---------------- STORAGE CLIP (a "C" ring, vertical axis, attached to one side) ----------------
if ADD_CLIP:
    ring_outer = trimesh.creation.cylinder(radius=clip_outer_r, height=CLIP_BAND, sections=64)
    ring_inner = trimesh.creation.cylinder(radius=clip_inner_r, height=CLIP_BAND + 2, sections=64)
    ring = ring_outer.difference(ring_inner, engine="manifold")

    # cut the opening gap out of the ring so it can flex open onto the handle
    gap_box = trimesh.creation.box(extents=[clip_outer_r * 3, clip_outer_r * 3, CLIP_BAND + 2])
    gap_box.apply_translation((clip_outer_r * 1.5, 0, 0))
    half_gap = math.radians(CLIP_GAP_DEG / 2)
    gap_box.apply_transform(trimesh.transformations.rotation_matrix(half_gap, [0, 0, 1]))
    ring = ring.difference(gap_box, engine="manifold")

    # position: gap facing outward (away from the foot, +X), ring pushed into the foot's -X side face
    ring.apply_translation((-(hw + clip_outer_r - CLIP_OVERLAP), 0, CLIP_Z))

    foot = foot.union(ring, engine="manifold")

foot.export("/home/claude/amp_feet/orange_crush_bass100_front_foot.stl")
print("Exported STL. Watertight:", foot.is_watertight)
print("Bounding box (mm):", foot.bounding_box.extents)
