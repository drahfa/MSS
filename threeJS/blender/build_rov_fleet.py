"""Build five original low-poly ROV class models and export browser-ready GLBs.

Run with:
    blender --background --python blender/build_rov_fleet.py

The attached class-comparison image is used only for broad class silhouettes.
Geometry is intentionally original and manufacturer-neutral.
"""

import bpy
import math
import os
from mathutils import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.dirname(HERE)
OUTPUT = os.path.join(PROJECT, "public", "models")
BLEND_FILE = os.path.join(HERE, "rov-fleet.blend")
os.makedirs(OUTPUT, exist_ok=True)

bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
for block in bpy.data.materials:
    bpy.data.materials.remove(block)


def material(name, color, metallic=0.0, roughness=0.55, emission=None):
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = (*color, 1)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = (*color, 1)
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = roughness
    if emission:
        bsdf.inputs["Emission Color"].default_value = (*emission, 1)
        bsdf.inputs["Emission Strength"].default_value = 3.0
    return mat


YELLOW = material("Safety yellow", (0.96, 0.58, 0.025), 0.18, 0.4)
YELLOW_DARK = material("Ochre panels", (0.58, 0.28, 0.02), 0.22, 0.5)
BLACK = material("Thruster black", (0.018, 0.026, 0.028), 0.45, 0.32)
GRAPHITE = material("Graphite frame", (0.055, 0.085, 0.09), 0.62, 0.3)
STEEL = material("Brushed steel", (0.42, 0.53, 0.55), 0.8, 0.26)
GLASS = material("Optical glass", (0.015, 0.09, 0.12), 0.3, 0.12)
LENS = material("Camera lens", (0.008, 0.025, 0.03), 0.65, 0.08, (0.02, 0.25, 0.32))
WHITE = material("Work light", (0.82, 0.93, 0.9), 0.1, 0.2, (0.52, 0.9, 0.82))
RED = material("Hydraulic red", (0.55, 0.045, 0.02), 0.38, 0.36)


def root(name):
    obj = bpy.data.objects.new(name, None)
    bpy.context.collection.objects.link(obj)
    obj.empty_display_type = "CUBE"
    obj["rov_class"] = name
    return obj


def finish(obj, parent, mat, bevel=0.0):
    obj.data.materials.append(mat)
    obj.parent = parent
    if bevel:
        bpy.context.view_layer.objects.active = obj
        obj.select_set(True)
        bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
        mod = obj.modifiers.new("Edge softening", "BEVEL")
        mod.width = bevel
        mod.segments = 2
        bpy.ops.object.modifier_apply(modifier=mod.name)
        obj.select_set(False)
    return obj


def cube(name, loc, size, parent, mat, bevel=0.04, rot=(0, 0, 0)):
    bpy.ops.mesh.primitive_cube_add(location=loc, rotation=rot)
    obj = bpy.context.object
    obj.name = name
    obj.scale = (size[0] / 2, size[1] / 2, size[2] / 2)
    return finish(obj, parent, mat, bevel)


def cylinder(name, loc, radius, depth, parent, mat, rot=(0, 0, 0), vertices=16, bevel=0.015):
    bpy.ops.mesh.primitive_cylinder_add(vertices=vertices, radius=radius, depth=depth, location=loc, rotation=rot)
    obj = bpy.context.object
    obj.name = name
    return finish(obj, parent, mat, bevel)


def sphere(name, loc, scale, parent, mat, segments=20, rings=12):
    bpy.ops.mesh.primitive_uv_sphere_add(segments=segments, ring_count=rings, location=loc)
    obj = bpy.context.object
    obj.name = name
    obj.scale = scale
    return finish(obj, parent, mat, 0.01)


def torus(name, loc, major, minor, parent, mat, rot=(0, 0, 0)):
    bpy.ops.mesh.primitive_torus_add(major_radius=major, minor_radius=minor, major_segments=16, minor_segments=6, location=loc, rotation=rot)
    obj = bpy.context.object
    obj.name = name
    return finish(obj, parent, mat)


def beam(name, a, b, radius, parent, mat):
    a, b = Vector(a), Vector(b)
    direction = b - a
    obj = cylinder(name, (a + b) / 2, radius, direction.length, parent, mat, vertices=10, bevel=0.01)
    obj.rotation_mode = "QUATERNION"
    obj.rotation_quaternion = direction.to_track_quat("Z", "Y")
    return obj


def thruster(name, loc, scale, parent, axis="Y", channel=0):
    directions = {"Y": Vector((0, -1, 0)), "X": Vector((1, 0, 0)), "Z": Vector((0, 0, 1))}
    direction = directions[axis] if isinstance(axis, str) else Vector(axis).normalized()
    rot = direction.to_track_quat("Z", "Y").to_euler()
    # Keep the duct open so the working propeller is visible from either end.
    torus(f"{name}_housing", loc, 0.185 * scale, 0.052 * scale, parent, GRAPHITE, rot)
    torus(f"{name}_duct", loc, 0.17 * scale, 0.024 * scale, parent, YELLOW_DARK, rot)

    rotor = bpy.data.objects.new(f"rotor_{name}", None)
    bpy.context.collection.objects.link(rotor)
    rotor.parent = parent
    rotor.location = loc
    rotor.rotation_euler = rot
    rotor["channel_index"] = channel
    rotor["thruster_axis_blender"] = axis.lower() if isinstance(axis, str) else tuple(round(v, 4) for v in direction)
    rotor["spin_axis_blender"] = "local_z"
    rotor["spin_axis_gltf"] = "local_y"

    # The rotor's local Z axis follows the thruster axis. All children are
    # authored in local space so Three.js can animate this one empty cleanly.
    hub = cylinder(f"{name}_propeller_hub", (0, 0, 0), 0.052 * scale, 0.09 * scale, rotor, STEEL, vertices=14, bevel=0.008)
    hub.location = (0, 0, 0)
    hub.rotation_euler = (0, 0, 0)

    blade_inner = 0.042 * scale
    blade_outer = 0.145 * scale
    blade_root = 0.025 * scale
    blade_tip = 0.056 * scale
    blade_thickness = 0.012 * scale
    verts = [
        (blade_inner, -blade_root, -blade_thickness),
        (blade_inner, blade_root, -blade_thickness),
        (blade_outer, blade_tip, -blade_thickness),
        (blade_outer, -blade_tip * 0.42, -blade_thickness),
        (blade_inner, -blade_root, blade_thickness),
        (blade_inner, blade_root, blade_thickness),
        (blade_outer, blade_tip, blade_thickness),
        (blade_outer, -blade_tip * 0.42, blade_thickness),
    ]
    faces = [
        (0, 1, 2, 3), (4, 7, 6, 5),
        (0, 4, 5, 1), (1, 5, 6, 2),
        (2, 6, 7, 3), (3, 7, 4, 0),
    ]
    for index in range(4):
        mesh = bpy.data.meshes.new(f"{name}_blade_{index}_mesh")
        mesh.from_pydata(verts, [], faces)
        mesh.update()
        blade = bpy.data.objects.new(f"{name}_propeller_blade_{index}", mesh)
        bpy.context.collection.objects.link(blade)
        blade.parent = rotor
        blade.rotation_euler = (math.radians(18), 0, index * math.pi / 2)
        blade.data.materials.append(YELLOW)
    return rotor


def camera_eye(name, loc, radius, parent):
    cylinder(f"{name}_rim", loc, radius * 1.2, radius * .45, parent, STEEL, (math.pi / 2, 0, 0), 16, .015)
    cylinder(name, (loc[0], loc[1] - radius * .25, loc[2]), radius, radius * .18, parent, LENS, (math.pi / 2, 0, 0), 18, .005)


def lamp(name, loc, parent, size=.08):
    cylinder(f"{name}_case", loc, size * 1.25, size * .45, parent, GRAPHITE, (math.pi / 2, 0, 0), 12, .01)
    cylinder(name, (loc[0], loc[1] - size * .26, loc[2]), size, size * .12, parent, WHITE, (math.pi / 2, 0, 0), 12, 0)


def frame_box(prefix, dims, parent, radius=.035, mat=GRAPHITE, z=0):
    x, y, h = dims[0] / 2, dims[1] / 2, dims[2] / 2
    corners = [(sx*x, sy*y, z+sz*h) for sx in (-1,1) for sy in (-1,1) for sz in (-1,1)]
    for sx in (-1,1):
        for sy in (-1,1): beam(f"{prefix}_v_{sx}_{sy}", (sx*x,sy*y,z-h),(sx*x,sy*y,z+h),radius,parent,mat)
    for sz in (-1,1):
        for sy in (-1,1): beam(f"{prefix}_x_{sz}_{sy}", (-x,sy*y,z+sz*h),(x,sy*y,z+sz*h),radius,parent,mat)
        for sx in (-1,1): beam(f"{prefix}_y_{sz}_{sx}", (sx*x,-y,z+sz*h),(sx*x,y,z+sz*h),radius,parent,mat)
    return corners


def claw(prefix, base, side, parent, scale=1):
    x, y, z = base
    elbow = (x + side * .18 * scale, y - .48 * scale, z - .12 * scale)
    wrist = (x + side * .30 * scale, y - .83 * scale, z - .03 * scale)
    beam(f"{prefix}_upper", base, elbow, .055*scale, parent, YELLOW_DARK)
    sphere(f"{prefix}_joint", elbow, (.09*scale,)*3, parent, STEEL, 12, 8)
    beam(f"{prefix}_forearm", elbow, wrist, .045*scale, parent, STEEL)
    for spread in (-1, 1):
        tip = (wrist[0] + spread * .09 * scale, wrist[1] - .16 * scale, wrist[2])
        beam(f"{prefix}_finger_{spread}", wrist, tip, .025*scale, parent, GRAPHITE)


def hobbyist():
    r = root("Hobbyist class")
    # Twin buoyancy lobes leave a genuine open lift tunnel through the CG.
    for x in (-.30, .30):
        sphere(f"streamlined_hull_{x}", (x, 0, 0), (.16, .78, .17), r, YELLOW, 24, 12)
        cube(f"keel_{x}", (x, .08, -.16), (.18, .74, .10), r, GRAPHITE, .035)
    for x in (-.27, .27): camera_eye(f"camera_{x}", (x, -.69, -.02), .075, r)
    for channel, x in enumerate((.43, -.43)): thruster(f"drive_{x}", (x, .15, -.02), .72, r, "Y", channel)
    thruster("depth", (0, .12, .05), .54, r, "Z", 2)
    cube("tail_fin", (0, .55, .15), (.06, .32, .22), r, YELLOW_DARK, .02)
    lamp("nose_light", (0, -.72, .1), r, .06)
    return r


def observation():
    r = root("Observation class")
    cube("pressure_body", (0, 0, .05), (1.05, 1.20, .56), r, YELLOW, .16)
    cylinder("front_port", (0, -.62, .06), .30, .14, r, GLASS, (math.pi/2,0,0), 20, .035)
    camera_eye("observation_camera", (0, -.71, .04), .11, r)
    for channel, x in enumerate((.56, -.56)):
        cylinder(f"float_{x}", (x, .04, .22), .16, .98, r, YELLOW_DARK, (math.pi/2,0,0), 14, .04)
        thruster(f"horizontal_{x}", (x, .24, -.16), .76, r, "Y", channel)
        # Lift ducts are outboard of both the pressure body and yellow floats.
        thruster(f"vertical_{x}", (math.copysign(.88, x), -.12, .18), .60, r, "Z", channel + 2)
    frame_box("obs_guard", (1.42, 1.34, .74), r, .025, STEEL, .02)
    for x in (-.28,.28): lamp(f"obs_lamp_{x}",(x,-.70,.20),r,.065)
    return r


def inspection():
    r = root("Inspection class")
    frame_box("inspection_frame", (1.65, 1.35, 1.30), r, .045, GRAPHITE, 0)
    cube("top_float", (0, .05, .56), (1.34, 1.02, .22), r, YELLOW, .07)
    cube("electronics", (0, .14, .05), (.72, .72, .45), r, GRAPHITE, .09)
    camera_eye("inspection_camera", (0, -.73, .02), .16, r)
    for x in (-.48,.48):
        lamp(f"inspection_lamp_{x}",(x,-.72,.24),r,.09)
    # Blender +X is vehicle port when vehicle faces -Y (180 deg Y rotation at runtime).
    horizontal = [
        (.48, -.34, (-1, -1, 0)), (-.48, -.34, (1, -1, 0)),
        (.48, .34, (1, -1, 0)), (-.48, .34, (-1, -1, 0)),
    ]
    for channel, (x, y, direction) in enumerate(horizontal):
        thruster(f"inspection_h{channel + 1}", (x, y, .02), .66, r, direction, channel)
    channel = 4
    for y in (-.43, .43):
        for x in (.86, -.86):
            thruster(f"inspection_v{channel - 3}", (x, y, .43), .52, r, "Z", channel)
            channel += 1
    sphere("sonar", (0,-.60,-.37),(.13,.13,.13),r,YELLOW_DARK,12,8)
    return r


def light_work():
    r = root("Light work class")
    frame_box("light_frame", (2.45, 1.72, 1.72), r, .07, GRAPHITE, 0)
    cube("buoyancy", (0, .08, .68), (2.18, 1.42, .42), r, YELLOW, .10)
    for x in (-.72,0,.72): cube(f"vent_{x}",(x,-.66,.69),(.42,.05,.18),r,BLACK,.015)
    cube("control_bay", (0,.20,.05),(1.25,.82,.68),r,YELLOW_DARK,.08)
    horizontal = [
        (.86, -.46, (-1, -1, 0)), (-.86, -.46, (1, -1, 0)),
        (.86, .46, (1, -1, 0)), (-.86, .46, (-1, -1, 0)),
    ]
    for channel, (x, y, direction) in enumerate(horizontal):
        thruster(f"light_h{channel + 1}", (x, y, .10), .95, r, direction, channel)
    for channel, x in enumerate((1.30, -1.30), start=4):
        thruster(f"light_v{channel - 3}", (x, -.08, .42), .82, r, "Z", channel)
        lamp(f"light_lamp_{x}",(x*.72,-.88,.28),r,.11)
    claw("arm_left",(-.58,-.67,-.35),-1,r,.85)
    claw("arm_right",(.58,-.67,-.35),1,r,.85)
    cube("tool_basket",(0,-.48,-.64),(1.24,.52,.16),r,STEEL,.03)
    return r


def heavy_work():
    r = root("Heavy work class")
    frame_box("heavy_frame", (4.5, 3.1, 2.75), r, .105, STEEL, -.15)
    for x in (-1.45,-.48,.48,1.45):
        cube(f"float_block_{x}",(x,.10,1.22),(.82,2.64,.55),r,YELLOW,.10)
    cube("power_module",(0,.44,.22),(2.5,1.15,.92),r,GRAPHITE,.10)
    cube("tooling_skid",(0,-.02,-1.38),(3.85,2.55,.24),r,STEEL,.04)
    horizontal = [
        (1.72, -.58, (-1, -1, 0)), (-1.72, -.58, (1, -1, 0)),
        (1.72, .58, (1, -1, 0)), (-1.72, .58, (-1, -1, 0)),
    ]
    for channel, (x, y, direction) in enumerate(horizontal):
        thruster(f"heavy_h{channel + 1}", (x, y, .04), 1.28, r, direction, channel)
    channel = 4
    for y in (-.72, .72):
        for x in (2.12, -2.12):
            # Outboard mount keeps the entire vertical flow column clear.
            thruster(f"heavy_v{channel - 3}", (x, y, .62), 1.12, r, "Z", channel)
            beam(f"heavy_v_mount_{channel - 3}", (math.copysign(1.72, x), y, .62), (x, y, .62), .06, r, STEEL)
            channel += 1
    for x in (-1.72,1.72):
        lamp(f"heavy_lamp_{x}",(x*.78,-1.60,.42),r,.15)
    claw("heavy_arm_left",(-1.05,-1.24,-.55),-1,r,1.35)
    claw("heavy_arm_right",(1.05,-1.24,-.55),1,r,1.35)
    beam("tower",(1.28,.40,1.48),(1.28,.40,3.08),.10,r,STEEL)
    cube("tether_head",(1.28,.40,3.14),(.50,.54,.34),r,YELLOW_DARK,.06)
    cylinder("hydraulic_pack",(0,.92,-.52),.34,1.15,r,RED,(0,math.pi/2,0),16,.04)
    return r


MODELS = {
    "hobbyist": hobbyist(),
    "observation": observation(),
    "inspection": inspection(),
    "light-work": light_work(),
    "heavy-work": heavy_work(),
}


def descendants(obj):
    result = [obj]
    for child in obj.children:
        result.extend(descendants(child))
    return result


for key, model in MODELS.items():
    bpy.ops.object.select_all(action="DESELECT")
    for obj in descendants(model): obj.select_set(True)
    bpy.context.view_layer.objects.active = model
    bpy.ops.export_scene.gltf(
        filepath=os.path.join(OUTPUT, f"rov-{key}.glb"),
        export_format="GLB",
        use_selection=True,
        export_apply=True,
        export_yup=True,
        export_extras=True,
        export_cameras=False,
        export_lights=False,
    )

# Arrange the source scene as a fleet lineup after exporting centred assets.
for model, x in zip(MODELS.values(), (-8.0, -4.8, -1.4, 2.7, 8.5)):
    model.location.x = x

bpy.ops.object.camera_add(location=(14, -24, 12), rotation=(math.radians(66), 0, math.radians(30)))
bpy.context.object.name = "Fleet preview camera"
bpy.context.scene.camera = bpy.context.object
bpy.ops.object.light_add(type="AREA", location=(0, -7, 13))
bpy.context.object.data.energy = 1800
bpy.context.object.data.shape = "RECTANGLE"
bpy.context.object.data.size = 14
bpy.ops.object.light_add(type="AREA", location=(-10, -4, 5))
bpy.context.object.data.energy = 900
bpy.context.object.data.color = (0.45, 0.75, 1.0)
bpy.context.scene.render.engine = "BLENDER_EEVEE_NEXT"
bpy.context.scene.world.color = (0.012, 0.022, 0.03)
bpy.ops.wm.save_as_mainfile(filepath=BLEND_FILE)

print(f"Built {len(MODELS)} ROV classes in {OUTPUT}")
