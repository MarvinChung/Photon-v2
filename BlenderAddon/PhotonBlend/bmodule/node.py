from ..utility import settings

import bpy
import nodeitems_utils

from abc import abstractmethod


class PhMaterialNodeTree(bpy.types.NodeTree):

	bl_idname = "PH_MATERIAL_NODE_TREE"
	bl_label  = "Photon Node Tree"
	bl_icon   = "MATERIAL"

	COMPATIBLE_ENGINES = {settings.renderer_id_name}

	@classmethod
	def poll(cls, b_context):
		render_settings = b_context.scene.render
		return render_settings.engine in cls.COMPATIBLE_ENGINES


class PhMaterialNodeSocket(bpy.types.NodeSocketShader):

	bl_idname = "PH_MATERIAL_NODE_SOCKET"
	bl_label  = "Photon Socket"

	def __init__(self):
		super().__init__()

	# Blender: draw socket's color
	def draw_color(self, b_context, node):
		return [0.0, 0.0, 0.0, 1.0]

	# Blender: draw socket
	def draw(self, b_context, b_layout, node, text):
		if self.is_linked or self.is_output:
			b_layout.label(text)
		else:
			row = b_layout.row()
			row.label(text)
			if node.inputs[text].default_value is not None:
				row.prop(node.inputs[text], "default_value")


class PhMaterialNode(bpy.types.Node):

	bl_idname = "PH_MATERIAL_NODE"
	bl_label  = "Photon Node"
	bl_icon   = "MATERIAL"

	# Blender: draw the buttons in node
	def draw_buttons(self, b_context, b_layout):
		pass


class PhMaterialNodeRealSocket(PhMaterialNodeSocket):

	bl_idname = "PH_MATERIAL_NODE_FLOAT_SOCKET"
	bl_label  = "Photon Real Socket"

	default_value = bpy.props.FloatProperty(name = "Real", default=0.0, min=0.0, max=1.0)

	def __init__(self):
		super().__init__()

	def draw_color(self, b_context, node):
		return [0.5, 0.5, 0.5, 1.0]  # gray


class PhSurfaceMaterialCategory(nodeitems_utils.NodeCategory):

	@classmethod
	def poll(cls, b_context):
		return b_context.space_data.tree_type == PhMaterialNodeTree.bl_idname


class PhDiffuseSurfaceNode(PhMaterialNode):

	bl_idname = "PH_DIFFUSE_SURFACE"
	bl_label  = "Diffuse Surface"

	diffusion_type = bpy.props.EnumProperty(
		items = [
			("LAMBERTIAN", "Lambertian", "")
		],
		name        = "Diffusion Type",
		description = "surface diffusion types",
		default     = "LAMBERTIAN"
	)

	# Blender: draw the buttons in node
	def draw_buttons(self, b_context, b_layout):
		row = b_layout.row()
		row.prop(self, "diffusion_type")


PH_MATERIAL_NODE_SOCKETS = [
	PhMaterialNodeRealSocket
]


PH_MATERIAL_NODES = [
	PhDiffuseSurfaceNode
]


PH_MATERIAL_NODE_CATEGORIES = [
	PhSurfaceMaterialCategory("SURFACE_MATERIAL", "Surface Material", items = [
		nodeitems_utils.NodeItem(PhDiffuseSurfaceNode.bl_idname)
	])
]


def register():

	bpy.utils.register_class(PhMaterialNodeTree)

	for socket_type in PH_MATERIAL_NODE_SOCKETS:
		bpy.utils.register_class(socket_type)

	for node_type in PH_MATERIAL_NODES:
		bpy.utils.register_class(node_type)

	nodeitems_utils.register_node_categories("PH_MATERIAL_NODE_CATEGORIES", PH_MATERIAL_NODE_CATEGORIES)


def unregister():

	bpy.utils.unregister_class(PhMaterialNodeTree)

	for socket_type in PH_MATERIAL_NODE_SOCKETS:
		bpy.utils.unregister_class(socket_type)

	for node_type in PH_MATERIAL_NODES:
		bpy.utils.unregister_class(node_type)

	nodeitems_utils.unregister_node_categories("PH_MATERIAL_NODE_CATEGORIES")


