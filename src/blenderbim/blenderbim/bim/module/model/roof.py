# BlenderBIM Add-on - OpenBIM Blender Add-on
# Copyright (C) 2023 Dion Moult <dion@thinkmoult.com>, @Andrej730
#
# This file is part of BlenderBIM Add-on.
#
# BlenderBIM Add-on is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# BlenderBIM Add-on is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with BlenderBIM Add-on.  If not, see <http://www.gnu.org/licenses/>.

import bpy
from bpy.types import Operator
import bmesh

import ifcopenshell
import blenderbim
import blenderbim.tool as tool
from blenderbim.bim.helper import convert_property_group_from_si
from blenderbim.bim.module.model.door import bm_sort_out_geom
from blenderbim.bim.module.model.data import RoofData, refresh
from blenderbim.bim.module.model.decorator import ProfileDecorator

import json
from math import tan, radians, degrees, atan
from mathutils import Vector, Matrix
from bpypolyskel import bpypolyskel
import shapely
from pprint import pprint
from itertools import chain

# reference:
# https://ifc43-docs.standards.buildingsmart.org/IFC/RELEASE/IFC4x3/HTML/lexical/IfcRoof.htm
# https://ifc43-docs.standards.buildingsmart.org/IFC/RELEASE/IFC4x3/HTML/lexical/IfcRoofType.htm


# create read only property in blender operator

def float_is_zero(f):
    return 0.0001 >= f >= - 0.0001

# TODO: move to generate_gable_roof
class GenerateHippedRoof(bpy.types.Operator, tool.Ifc.Operator):
    bl_idname = "bim.generate_hipped_roof"
    bl_label = "Generate Hipped Roof"
    bl_options = {"REGISTER", "UNDO"}
    
    roof_generation_methods = (
        ("HEIGHT", "HEIGHT", ""),
        ("ANGLE", "ANGLE", ""),
    )

    mode: bpy.props.EnumProperty(
        name="Roof Generation Method", items=roof_generation_methods, default="ANGLE"
    )
    height: bpy.props.FloatProperty(default=1)
    angle: bpy.props.FloatProperty(default=45) # TODO: RAD
    resulting_angle: bpy.props.FloatProperty(default=0)

    def _execute(self, context):
        obj = bpy.context.active_object
        if not obj:
            self.report({"ERROR"}, "Need to select some object first.")
            return {"CANCELLED"}

        bm = tool.Blender.get_bmesh_for_mesh(obj.data)
        # argument values are the defaults for `bpy.ops.mesh.dissolve_limited`
        bmesh.ops.dissolve_limit(bm, angle_limit=0.0872665, use_dissolve_boundaries=False, delimit={"NORMAL"}, edges=bm.edges[:], verts=bm.verts[:])
        bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=0.0001)
        tool.Blender.apply_bmesh(obj.data, bm)

        generated_roof_angle = generate_gable_roof(obj, self.mode, self.height, self.angle, leave_footprint=True)
        self.resulting_angle = generated_roof_angle
        return {"FINISHED"}


def generate_hipped_roof(obj, mode="ANGLE", height=1.0, angle=10):
    boundary_lines = []

    for edge in obj.data.edges:
        boundary_lines.append(
            shapely.LineString([obj.data.vertices[edge.vertices[0]].co, obj.data.vertices[edge.vertices[1]].co])
        )

    unioned_boundaries = shapely.union_all(shapely.GeometryCollection(boundary_lines))
    closed_polygons = shapely.polygonize(unioned_boundaries.geoms)
    
    # find the polygon with the biggest area
    roof_polygon = max(closed_polygons.geoms, key=lambda polygon: polygon.area)

    # add z coordinate if not present
    roof_polygon = shapely.force_3d(roof_polygon)

    # make sure the polygon is counter-clockwise
    if not shapely.is_ccw(roof_polygon):
        roof_polygon = roof_polygon.reverse()

    # Define vertices for the base footprint of the building at height 0.0
    # counterclockwise order
    verts = [Vector(v) for v in roof_polygon.exterior.coords[0:-1]]
    total_exterior_verts = len(verts)
    next_index = total_exterior_verts

    inner_loops = None # in case when there is no .interiors
    for interior in roof_polygon.interiors:
        if inner_loops is None:
            inner_loops = []
        loop = interior.coords[0:-1]
        total_verts = len(loop)
        verts.extend([Vector(v) for v in loop])
        inner_loops.append((next_index, total_verts))
        next_index += total_verts

    unit_vectors = None  # we have no unit vectors, let them computed by polygonize()
    start_exterior_index = 0

    faces = []

    if mode == "HEIGHT":
        height = height
        angle = 0.0
    else:
        angle = tan(radians(round(angle, 4)))
        height = 0.0

    faces = bpypolyskel.polygonize(
        verts, start_exterior_index, total_exterior_verts, inner_loops, height, angle, faces, unit_vectors
    )

    edges = []

    bm = tool.Blender.get_bmesh_for_mesh(obj.data, clean=True)
    new_verts = [bm.verts.new(v) for v in verts]
    new_edges = [bm.edges.new([new_verts[vi] for vi in edge]) for edge in edges]
    new_faces = [bm.faces.new([new_verts[vi] for vi in face]) for face in faces]

    extrusion_geom = bmesh.ops.extrude_face_region(bm, geom=bm.faces)["geom"]
    extruded_verts = bm_sort_out_geom(extrusion_geom)["verts"]
    bmesh.ops.translate(bm, vec=[0.0, 0.0, 0.1], verts=extruded_verts)

    tool.Blender.apply_bmesh(obj.data, bm)

def generate_gable_roof(obj, mode="ANGLE", height=1.0, angle=10, leave_footprint=False):
    debug_z_distance = 1.0 if leave_footprint else 0.0
    boundary_lines = []

    bm = tool.Blender.get_bmesh_for_mesh(obj.data)

    # identifying footprint verts by min_z
    footprint_verts = []
    footprint_min_z = 0
    for v in bm.verts:
        if float_is_zero(v.co.z - footprint_min_z):
            footprint_verts.append(v)

        elif footprint_min_z > v.co.z:
            footprint_min_z = v.co.z
            footprint_verts = [v]

        # ignore vertices above min_z
    
    # python bmesh recalculate vertices index after removing some

    # TODO: possibly if we plan to remove all unnecessary geometry
    # then all edges are footprint edges
    footprint_edges = list(set(chain(*[v.link_edges for v in footprint_verts])))

    for v in bm.verts:
        if v not in footprint_verts:
            bm.verts.remove(v)

    original_geometry_data = dict()
    # saving edges as sets to compare later
    crease_level = bm.edges.layers.crease.verify()
    original_geometry_data['edges'] = [set(bm_get_indices(e.verts)) for e in footprint_edges]
    original_geometry_data['verts'] = {v.index:v.co.copy() for v in footprint_verts}
    original_geometry_data['crease_values'] = [e[crease_level] for e in footprint_edges]

    print(len(footprint_verts))
    print("original_geometry_data['edges']", original_geometry_data['edges'])

    # assume all vertices are on the same level

    for edge in footprint_edges:
        boundary_lines.append(
            shapely.LineString([v.co for v in edge.verts])
        )

    if not leave_footprint:
        bm.clear()

    tool.Blender.apply_bmesh(obj.data, bm)

    unioned_boundaries = shapely.union_all(shapely.GeometryCollection(boundary_lines))
    closed_polygons = shapely.polygonize(unioned_boundaries.geoms)
    
    # find the polygon with the biggest area
    roof_polygon = max(closed_polygons.geoms, key=lambda polygon: polygon.area)

    # add z coordinate if not present
    roof_polygon = shapely.force_3d(roof_polygon)

    # make sure the polygon is counter-clockwise
    if not shapely.is_ccw(roof_polygon):
        roof_polygon = roof_polygon.reverse()

    # Define vertices for the base footprint of the building at height 0.0
    # counterclockwise order
    verts = [Vector(v) for v in roof_polygon.exterior.coords[0:-1]]
    total_exterior_verts = len(verts)
    next_index = total_exterior_verts

    inner_loops = None # in case when there is no .interiors
    for interior in roof_polygon.interiors:
        if inner_loops is None:
            inner_loops = []
        loop = interior.coords[0:-1]
        total_verts = len(loop)
        verts.extend([Vector(v) for v in loop])
        inner_loops.append((next_index, total_verts))
        next_index += total_verts

    unit_vectors = None  # we have no unit vectors, let them computed by polygonize()
    start_exterior_index = 0

    faces = []

    if mode == "HEIGHT":
        height = height
        angle = 0.0
    else:
        angle = tan(radians(round(angle, 4)))
        height = 0.0

    faces = bpypolyskel.polygonize(
        verts, start_exterior_index, total_exterior_verts, inner_loops, height, angle, faces, unit_vectors
    )

    edges = []
    verts = [v + Vector((0.0, 0.0, debug_z_distance)) for v in verts]
    bm = tool.Blender.get_bmesh_for_mesh(obj.data)
    new_verts = [bm.verts.new(v) for v in verts]
    new_edges = [bm.edges.new([new_verts[vi] for vi in edge]) for edge in edges]
    new_faces = [bm.faces.new([new_verts[vi] for vi in face]) for face in faces]

    # TODO: uncomment after debug
    # TODO: and match base_edges with extruded ones...
    # extrusion_geom = bmesh.ops.extrude_face_region(bm, geom=bm.faces)["geom"]
    # extruded_verts = bm_sort_out_geom(extrusion_geom)["verts"]
    # bmesh.ops.translate(bm, vec=[0.0, 0.0, 0.1], verts=extruded_verts)

    tool.Blender.apply_bmesh(obj.data, bm)

    # trying to match edges from new mesh
    # with the original meshes
    # to figure their crease values
    edges_match = {}
    # need to make sure edges are actually at z = 0
    footprint_edges = []
    verts_to_change = {}

    # TODO: match edges
    # TODO: match the opposite - take only edges with crease values
    # and try to match them to new ones

    def find_close_original_vert(co):
        for vi in original_geometry_data['verts']:
            old_co = original_geometry_data['verts'][vi]
            if float_is_zero( (co - Vector([0,0,debug_z_distance]) - old_co).length):
                return vi
            
    for edge in obj.data.edges:
        edge_verts = [obj.data.vertices[vi].co for vi in edge.vertices]
        if all( float_is_zero(v.z - debug_z_distance - footprint_min_z) for v in edge_verts):
            footprint_edges.append(edge)

    # TODO: refactor with bmesh
    def find_other_polygon_verts_i(edge):
        for polygon in obj.data.polygons:
            polygon_verts = polygon.vertices[:]
            if all(vi in polygon_verts for vi in edge.vertices):
                break
        other_verts_i = [vi for vi in polygon.vertices if vi not in edge.vertices[:]]
        return other_verts_i

    def angle_between(A, B, P):
        """angle between AB and CP where C is P projected on AB"""
        AP = P - A
        AB = B - A
        AB_dir = AB.normalized()
        proj_length = AP.dot(AB_dir)
        C = A + AB_dir * proj_length
        Pp = P * Vector([1, 1, 0]) + Vector([0, 0, C.z])
        angle_tan = (P.z-C.z) / (Pp-C).length
        return degrees(atan(angle_tan))

    def project_vert_on_edge_linearly(projected_vert_co, edge_verts_coords, t):
        """keeps the same z for `projected_vert_co`"""
        A, B = [v.xy for v in edge_verts_coords]
        O = projected_vert_co.copy()
        AB = B - A
        AB_dir = AB.normalized()
        AO = O.xy - A
        edge_space = Matrix( [AB_dir, AB_dir.yx * Vector([-1, 1])] ).transposed()
        AO_local = edge_space @ AO
        AO_local.y = AO_local.y * (1-t)
        AO = edge_space.inverted() @ AO_local
        transformed_co = (AO + A).to_3d() + Vector([0, 0, O.z])
        return transformed_co

    # find an angle between the first of the edge and its related vert
    # since all angle is the same across the all edges
    # TODO: change angle with average angle because it could be different across the roof
    angle_calculation_edge = footprint_edges[0]
    angle_calculation_edge_vertices = [obj.data.vertices[vi].co for vi in angle_calculation_edge.vertices]
    other_vert_i = find_other_polygon_verts_i(footprint_edges[0])[0]
    other_vert_co = obj.data.vertices[other_vert_i].co
    calculated_angle = angle_between(*angle_calculation_edge_vertices, other_vert_co)

    print('original geometry data', original_geometry_data['edges'])
    for edge in footprint_edges:
        print('base edge', edge, edge.vertices[:]) # TODO: remove after debug
        edge_verts = [obj.data.vertices[vi].co for vi in edge.vertices]

        # finding appropriate vert from the original geometry
        old_verts_notation = set(find_close_original_vert(v) for v in edge_verts)
        
        # TODO: remove after debug
        print('old verts notation', old_verts_notation)
        if old_verts_notation not in original_geometry_data['edges']:
            continue

        # if found edge match
        # then figuring it's crease value
        old_edge_index = original_geometry_data['edges'].index(old_verts_notation)
        old_edge_crease_value = original_geometry_data["crease_values"][old_edge_index]
        edges_match[edge] = old_edge_crease_value

        # TODO: remove after debug
        print(f"Found a match: {edge.vertices[:]}, {original_geometry_data['edges'].index(old_verts_notation)}, {original_geometry_data['crease_values'][old_edge_index]}")
        
        # if edge crease_value is non zero, then we move it's vertex
        if old_edge_crease_value != 0:
            old_edge_crease_value = abs(old_edge_crease_value)
            other_verts_i = find_other_polygon_verts_i(edge)
            for other_vert_i in other_verts_i:
                other_vert = obj.data.vertices[other_vert_i]
                top_vert_co = verts_to_change.get(other_vert_i, other_vert.co)
                verts_to_change[other_vert_i] = project_vert_on_edge_linearly(top_vert_co, edge_verts, old_edge_crease_value)

    bm = tool.Blender.get_bmesh_for_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    for vi in verts_to_change:
        print(f'moving vertex {vi}, {bm.verts[vi].co}')
        bm.verts[vi].co = verts_to_change[vi]
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    tool.Blender.apply_bmesh(obj.data, bm)

    return calculated_angle


def bm_get_indices(sequence):
    return [i.index for i in sequence]


def update_roof_modifier_ifc_data(context):
    obj = context.active_object
    props = obj.BIMRoofProperties
    element = tool.Ifc.get_entity(obj)

    # type attributes
    element.PredefinedType = props.roof_type
    # occurences attributes
    # occurences = tool.Ifc.get_all_element_occurences(element)

    # TODO: add Qto_RoofBaseQuantities, need to calculate GrossArea, NetArea, ProjectedArea
    # https://ifc43-docs.standards.buildingsmart.org/IFC/RELEASE/IFC4x3/HTML/lexical/Qto_RoofBaseQuantities.htm


def update_bbim_roof_pset(element, roof_data):
    pset = tool.Pset.get_element_pset(element, "BBIM_Roof")
    if not pset:
        pset = ifcopenshell.api.run("pset.add_pset", tool.Ifc.get(), product=element, name="BBIM_Roof")
    roof_data = json.dumps(roof_data, default=list)
    ifcopenshell.api.run("pset.edit_pset", tool.Ifc.get(), pset=pset, properties={"Data": roof_data})


def update_roof_modifier_bmesh(context):
    obj = context.object
    props = obj.BIMRoofProperties

    if not RoofData.is_loaded:
        RoofData.load()
    path_data = RoofData.data["parameters"]["data_dict"]["path_data"]

    si_conversion = ifcopenshell.util.unit.calculate_unit_scale(tool.Ifc.get())
    # need to make sure we support edit mode
    # since users will probably be in edit mode when they'll be changing roof path
    bm = tool.Blender.get_bmesh_for_mesh(obj.data, clean=True)

    # generating roof path
    new_verts = [bm.verts.new(Vector(v) * si_conversion) for v in path_data["verts"]]
    new_edges = [bm.edges.new((new_verts[e[0]], new_verts[e[1]])) for e in path_data["edges"]]

    if props.is_editing_path:
        tool.Blender.apply_bmesh(obj.data, bm)
        return

    # apply dissolve limit seems to get more correct results with `generate_hipped_roof`
    # argument values are the defaults for `bpy.ops.mesh.dissolve_limited`
    bmesh.ops.dissolve_limit(
        bm, angle_limit=0.0872665, use_dissolve_boundaries=False, delimit={"NORMAL"}, edges=bm.edges[:], verts=bm.verts[:]
    )
    tool.Blender.apply_bmesh(obj.data, bm)

    height = props.height * si_conversion
    angle = props.angle * si_conversion
    generation_method = props.generation_method
    generate_hipped_roof(obj, generation_method, height, angle)


def get_path_data(obj):
    si_conversion = ifcopenshell.util.unit.calculate_unit_scale(tool.Ifc.get())

    if obj.mode == "EDIT":
        # otherwise mesh may not contain all changes
        # added in edit mode
        obj.update_from_editmode()

    bm = tool.Blender.get_bmesh_for_mesh(obj.data)

    # remove internal edges and faces
    # adding missing faces so we could rely on `e.is_boundary` later
    bmesh.ops.contextual_create(bm, geom=bm.edges[:])

    edges_to_dissolve = [e for e in bm.edges if not e.is_boundary]
    bmesh.ops.dissolve_edges(bm, edges=edges_to_dissolve)
    bmesh.ops.delete(bm, geom=bm.faces[:], context="FACES_ONLY")

    path_data = dict()
    path_data["edges"] = [bm_get_indices(e.verts) for e in bm.edges]
    path_data["verts"] = [v.co / si_conversion for v in bm.verts]

    if not path_data["edges"] or not path_data["verts"]:
        return None
    return path_data


class BIM_OT_add_roof(bpy.types.Operator, tool.Ifc.Operator):
    bl_idname = "mesh.add_roof"
    bl_label = "Roof"
    bl_options = {"REGISTER", "UNDO"}

    def _execute(self, context):
        ifc_file = tool.Ifc.get()
        if not ifc_file:
            self.report({"ERROR"}, "You need to start IFC project first to create a roof.")
            return {"CANCELLED"}

        if context.object is not None:
            spawn_location = context.object.location.copy()
            context.object.select_set(False)
        else:
            spawn_location = bpy.context.scene.cursor.location.copy()

        mesh = bpy.data.meshes.new("IfcRoof")
        obj = bpy.data.objects.new("IfcRoof", mesh)
        obj.location = spawn_location
        body_context = ifcopenshell.util.representation.get_context(ifc_file, "Model", "Body", "MODEL_VIEW")
        blenderbim.core.root.assign_class(
            tool.Ifc,
            tool.Collector,
            tool.Root,
            obj=obj,
            ifc_class="IfcRoof",
            should_add_representation=True,
            context=body_context,
        )
        bpy.ops.object.select_all(action="DESELECT")
        bpy.context.view_layer.objects.active = None
        bpy.context.view_layer.objects.active = obj
        obj.select_set(True)
        bpy.ops.bim.add_roof()
        return {"FINISHED"}


# UI operators
class AddRoof(bpy.types.Operator, tool.Ifc.Operator):
    bl_idname = "bim.add_roof"
    bl_label = "Add Roof"
    bl_options = {"REGISTER"}

    def _execute(self, context):
        obj = context.active_object
        element = tool.Ifc.get_entity(obj)
        props = obj.BIMRoofProperties
        si_conversion = ifcopenshell.util.unit.calculate_unit_scale(tool.Ifc.get())

        if element.is_a() not in ("IfcRoof", "IfcRoofType"):
            self.report({"ERROR"}, "Object has to be IfcRoof/IfcRoofType type to add a roof.")
            return {"CANCELLED"}

        # need to make sure all default props will have correct units
        if not props.roof_added_previously:
            skip_props = ("is_editing", "roof_type", "roof_added_previously", "generation_method")
            convert_property_group_from_si(props, skip_props=skip_props)

        roof_data = props.get_general_kwargs()
        path_data = get_path_data(obj)
        if not path_data:
            path_data = {
                "edges": [[0, 1], [1, 2], [2, 3], [3, 0]],
                "verts": [
                    Vector([-5.0, -5.0, 0.0]) / si_conversion,
                    Vector([-5.0, 5.0, 0.0]) / si_conversion,
                    Vector([5.0, 5.0, 0.0]) / si_conversion,
                    Vector([5.0, -5.0, 0.0]) / si_conversion,
                ],
            }
        roof_data["path_data"] = path_data

        update_bbim_roof_pset(element, roof_data)
        update_roof_modifier_ifc_data(context)
        refresh()
        update_roof_modifier_bmesh(context)
        return {"FINISHED"}


class EnableEditingRoof(bpy.types.Operator, tool.Ifc.Operator):
    bl_idname = "bim.enable_editing_roof"
    bl_label = "Enable Editing Roof"
    bl_options = {"REGISTER"}

    def _execute(self, context):
        obj = context.active_object
        props = obj.BIMRoofProperties
        element = tool.Ifc.get_entity(obj)
        data = json.loads(ifcopenshell.util.element.get_pset(element, "BBIM_Roof", "Data"))
        data["path_data"] = json.dumps(data["path_data"])

        # required since we could load pset from .ifc and BIMRoofProperties won't be set
        for prop_name in data:
            setattr(props, prop_name, data[prop_name])

        # need to make sure all props that weren't used before
        # will have correct units
        skip_props = ("is_editing", "roof_type", "roof_added_previously", "generation_method")
        skip_props += tuple(data.keys())
        convert_property_group_from_si(props, skip_props=skip_props)

        props.is_editing = 1
        return {"FINISHED"}


class CancelEditingRoof(bpy.types.Operator, tool.Ifc.Operator):
    bl_idname = "bim.cancel_editing_roof"
    bl_label = "Cancel editing Roof"
    bl_options = {"REGISTER"}

    def _execute(self, context):
        obj = context.active_object
        element = tool.Ifc.get_entity(obj)
        data = json.loads(ifcopenshell.util.element.get_pset(element, "BBIM_Roof", "Data"))
        props = obj.BIMRoofProperties
        # restore previous settings since editing was canceled
        for prop_name in data:
            setattr(props, prop_name, data[prop_name])

        body = ifcopenshell.util.representation.get_representation(element, "Model", "Body", "MODEL_VIEW")
        blenderbim.core.geometry.switch_representation(
            tool.Ifc,
            tool.Geometry,
            obj=obj,
            representation=body,
            should_reload=True,
            is_global=True,
            should_sync_changes_first=False,
        )

        props.is_editing = -1
        return {"FINISHED"}


class FinishEditingRoof(bpy.types.Operator, tool.Ifc.Operator):
    bl_idname = "bim.finish_editing_roof"
    bl_label = "Finish editing roof"
    bl_options = {"REGISTER"}

    def _execute(self, context):
        obj = context.active_object
        element = tool.Ifc.get_entity(obj)
        props = obj.BIMRoofProperties

        if not RoofData.is_loaded:
            RoofData.load()
        path_data = RoofData.data["parameters"]["data_dict"]["path_data"]

        roof_data = props.get_general_kwargs()
        roof_data["path_data"] = path_data
        props.is_editing = -1

        update_bbim_roof_pset(element, roof_data)
        update_roof_modifier_ifc_data(context)
        return {"FINISHED"}


class EnableEditingRoofPath(bpy.types.Operator, tool.Ifc.Operator):
    bl_idname = "bim.enable_editing_roof_path"
    bl_label = "Enable Editing Roof Path"
    bl_options = {"REGISTER"}

    def _execute(self, context):
        obj = context.active_object
        props = obj.BIMRoofProperties

        props.is_editing_path = True
        update_roof_modifier_bmesh(context)

        if bpy.context.object.mode != "EDIT":
            bpy.ops.object.mode_set(mode="EDIT")
        bpy.ops.wm.tool_set_by_id(tool.Blender.get_viewport_context(), name="bim.cad_tool")

        def get_custom_bmesh():
            # copying to make sure not to mutate the edit mode bmesh
            main_bm = tool.Blender.get_bmesh_for_mesh(obj.data).copy()
            second_bm = bmesh.new()
            bmesh.ops.create_cube(second_bm)
            tool.Blender.bmesh_join(main_bm, second_bm)
            return main_bm

        ProfileDecorator.install(context, get_custom_bmesh, draw_faces=True)
        return {"FINISHED"}


class CancelEditingRoofPath(bpy.types.Operator, tool.Ifc.Operator):
    bl_idname = "bim.cancel_editing_roof_path"
    bl_label = "Cancel Editing Roof Path"
    bl_options = {"REGISTER"}

    def _execute(self, context):
        obj = context.active_object
        props = obj.BIMRoofProperties

        ProfileDecorator.uninstall()
        props.is_editing_path = False

        update_roof_modifier_bmesh(context)
        if bpy.context.object.mode == "EDIT":
            bpy.ops.object.mode_set(mode="OBJECT")
        return {"FINISHED"}


class FinishEditingRoofPath(bpy.types.Operator, tool.Ifc.Operator):
    bl_idname = "bim.finish_editing_roof_path"
    bl_label = "Finish Editing Roof Path"
    bl_options = {"REGISTER"}

    def _execute(self, context):
        obj = context.active_object
        element = tool.Ifc.get_entity(obj)
        props = obj.BIMRoofProperties

        roof_data = props.get_general_kwargs()
        path_data = get_path_data(obj)
        roof_data["path_data"] = path_data
        ProfileDecorator.uninstall()
        props.is_editing_path = False

        update_bbim_roof_pset(element, roof_data)
        refresh()  # RoofData has to be updated before run update_roof_modifier_bmesh
        update_roof_modifier_bmesh(context)
        if bpy.context.object.mode == "EDIT":
            bpy.ops.object.mode_set(mode="OBJECT")
        return {"FINISHED"}


class RemoveRoof(bpy.types.Operator, tool.Ifc.Operator):
    bl_idname = "bim.remove_roof"
    bl_label = "Remove Roof"
    bl_options = {"REGISTER"}

    def _execute(self, context):
        obj = context.active_object
        props = obj.BIMRoofProperties
        element = tool.Ifc.get_entity(obj)
        obj.BIMRoofProperties.is_editing = -1

        pset = tool.Pset.get_element_pset(element, "BBIM_Roof")
        ifcopenshell.api.run("pset.remove_pset", tool.Ifc.get(), pset=pset)
        props.roof_added_previously = True
        return {"FINISHED"}


class SetGableRoofEdgeAngle(bpy.types.Operator):
    bl_idname = "bim.set_gable_roof_edge_angle"
    bl_label = "Set gable roof edge angle"
    bl_options = {"REGISTER", "UNDO"}
    angle: bpy.props.FloatProperty(name="Angle", default=90)

    @classmethod
    def poll(cls, context):
        obj = context.active_object
        return obj and obj.type == "MESH" and context.mode == "EDIT_MESH"

    def draw(self, context):
        print('hello')
        layout = self.layout
        for prop in self.__class__.__annotations__.keys():
            layout.prop(self, prop)

    def execute(self, context):
        # tried to avoid bmesh with foreach_get and foreach_set
        # but in EDIT mode it's only possible to change attributes by working with bmesh

        me = context.object.data
        bm = tool.Blender.get_bmesh_for_mesh(me)

        # check if attribute exists or create one
        if 'BB_gable_roof_angles' not in me.attributes:
            me.attributes.new('BB_gable_roof_angles', type='FLOAT', domain='EDGE')

        angles_layer = bm.edges.layers.float['BB_gable_roof_angles']

        # TODO: reset previous value to 0 from invoke
        # set attribute to value from operator (angle=90)
        for e in bm.edges:
            if not e.select:
                continue
            e[angles_layer] = self.angle

        tool.Blender.apply_bmesh(me, bm)
        return {"FINISHED"}


def add_object_button(self, context):
    self.layout.operator(BIM_OT_add_roof.bl_idname, icon="PLUGIN")
