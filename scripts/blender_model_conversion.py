print("Script started")
import bpy
import os
from os.path import basename,dirname
import sys
print("Imports done")
# -----------------------
# Parse command line args
# -----------------------
argv = sys.argv
if "--" in argv:
    argv = argv[argv.index("--") + 1:]  # everything after --
else:
    argv = []

if len(argv) < 1:
    raise ValueError("Usage: blender --background --python script.py -- /path/to/input_dir")

input_mesh = os.path.abspath(argv[0])
input_dir = os.path.dirname(input_mesh)

# Output dirs
usd_dir = os.path.join(input_dir, "usd")
obj_dir = os.path.join(input_dir, "obj")
blend_dir = os.path.join(input_dir, "blend")
for d in (usd_dir, obj_dir, blend_dir):
    print(f"Ensuring: {d}")
    os.makedirs(d, exist_ok=True)

# -----------------------
# Clean scene
# -----------------------
print("Cleaning Scene...")
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete()

# -----------------------
# Import mesh
# -----------------------
print(f"Importing mesh... \"{input_mesh}\"")

# bpy.ops.preferences.addon_enable(module="io_mesh_ply")
# bpy.ops.import_mesh.ply(filepath=input_mesh)
bpy.ops.wm.ply_import(filepath=input_mesh)
obj = bpy.context.selected_objects[0]
obj.scale = (0.814, 0.814, 0.814)

# Delete all other objects
for o in bpy.data.objects:
    if o != obj:
        bpy.data.objects.remove(o, do_unlink=True)

# -----------------------
# Set origins
# -----------------------
print("Fixing Origin...")
bpy.context.view_layer.objects.active = obj
bpy.ops.object.origin_set(type='ORIGIN_CENTER_OF_MASS', center='BOUNDS')
bpy.ops.object.origin_set(type='GEOMETRY_ORIGIN')

# -----------------------
# Separate loose parts & keep largest
# -----------------------
print("Separating loose parts...")
bpy.ops.object.mode_set(mode='EDIT')
bpy.ops.mesh.separate(type='LOOSE')
bpy.ops.object.mode_set(mode='OBJECT')

largest = max(bpy.context.selected_objects, key=lambda o: len(o.data.vertices))
print(f"Separate parts: {len(bpy.context.selected_objects)}")
for o in bpy.context.selected_objects:
    if o != largest:
        bpy.data.objects.remove(o, do_unlink=True)

obj = largest

# -----------------------
# Add material with color attribute
# -----------------------
print("Material and color attributes...")
mat = bpy.data.materials.new(name="FusedMaterial")
mat.use_nodes = True
nodes = mat.node_tree.nodes
links = mat.node_tree.links

bsdf = nodes["Principled BSDF"]
color_attr = nodes.new("ShaderNodeVertexColor")
color_attr.layer_name = "Col" 
links.new(color_attr.outputs['Color'], bsdf.inputs['Base Color'])

obj.data.materials.append(mat)

# -----------------------
# Decimate modifier
# -----------------------
DEC_RATIO = 0.1
print(f"Applying Decimate... Ratio: {DEC_RATIO}")
dec = obj.modifiers.new("Decimate", 'DECIMATE')
dec.ratio = DEC_RATIO
bpy.ops.object.modifier_apply(modifier=dec.name)


# -----------------------
# UV unwrap
# -----------------------
print("UV Unwrap...")
# Ensure UV map exists
if not obj.data.uv_layers:
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.uv.smart_project()
    bpy.ops.object.mode_set(mode='OBJECT')

# Set the UV map as active
obj.data.uv_layers.active = obj.data.uv_layers[0]


# -----------------------
# Bake texture
# -----------------------
print("Baking texture...")
# Create new image for baking
img = bpy.data.images.new("BakedTex", width=2048, height=2048)

# Add image node & set active for baking
tex_node = nodes.new("ShaderNodeTexImage")
tex_node.image = img
nodes.active = tex_node

# Ensure correct object/material active
bpy.context.view_layer.objects.active = obj
obj.select_set(True)

# Set render engine to Cycles
bpy.context.scene.render.engine = 'CYCLES'

# Set scene to use GPU
bpy.context.scene.cycles.device = 'GPU'

# Configure bake to color only
bpy.context.scene.cycles.bake_type = 'DIFFUSE'
bpy.context.scene.render.bake.use_pass_direct = False
bpy.context.scene.render.bake.use_pass_indirect = False
bpy.context.scene.render.bake.use_pass_color = True

# Perform bake
bpy.ops.object.bake(type='DIFFUSE')

# Save baked texture
print("Save baked texture...")
tex_path = os.path.join(input_dir, "baked_texture.png")
img.filepath_raw = tex_path
img.file_format = 'PNG'
img.save()

# Connect baked texture to material
print("Connecting baked texture to material...")
links.new(tex_node.outputs['Color'], bsdf.inputs['Base Color'])

# -----------------------
# Save .blend
# -----------------------
print("Saving Blend file...")
blend_path = os.path.join(blend_dir, "fused_model.blend")
bpy.ops.wm.save_as_mainfile(filepath=blend_path)

# -----------------------
# Export USD & OBJ
# -----------------------
usd_path = os.path.join(usd_dir, "fused_model.usd")
obj_path = os.path.join(obj_dir, "fused_model.obj")

# https://docs.blender.org/api/current/bpy.ops.wm.html#bpy.ops.wm.usd_export 
bpy.ops.wm.usd_export(filepath=usd_path,check_existing=True)
# bpy.ops.export_scene.usd(filepath=usd_path, selected_objects=True)
# bpy.ops.export_scene.obj(filepath=obj_path, use_selection=True)
# https://docs.blender.org/api/current/bpy.ops.wm.html#bpy.ops.wm.usd_export
bpy.ops.wm.obj_export(filepath=obj_path,check_existing=True,path_mode="COPY")

print("✅ Processing complete!")
print(f"Blend: {blend_path}")
print(f"USD:   {usd_path}")
print(f"OBJ:   {obj_path}")
print(f"Tex:   {tex_path}")
