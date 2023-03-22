# BlenderBIM Add-on - OpenBIM Blender Add-on
# Copyright (C) 2020, 2021 Dion Moult <dion@thinkmoult.com>
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

import gpu
import bgl
import bmesh
import blenderbim.tool as tool
from math import pi, degrees, sin, cos, radians
from bpy.types import SpaceView3D
from mathutils import Vector, Matrix
from gpu.types import GPUShader, GPUBatch, GPUIndexBuf, GPUVertBuf, GPUVertFormat
from gpu_extras.batch import batch_for_shader


class ProfileDecorator:
    installed = None

    @classmethod
    def install(cls, context):
        if cls.installed:
            cls.uninstall()
        handler = cls()
        cls.installed = SpaceView3D.draw_handler_add(handler, (context,), "WINDOW", "POST_VIEW")

    @classmethod
    def uninstall(cls):
        try:
            SpaceView3D.draw_handler_remove(cls.installed, "WINDOW")
        except ValueError:
            pass
        cls.installed = None

    def __call__(self, context):
        obj = context.active_object
        if obj.mode != "EDIT":
            return

        # TODO: replace BGL with GPU
        bgl.glLineWidth(2)
        bgl.glPointSize(6)
        bgl.glEnable(bgl.GL_BLEND)
        bgl.glEnable(bgl.GL_LINE_SMOOTH)

        all_vertices = []
        error_vertices = []
        selected_vertices = []
        unselected_vertices = []
        # special = associated with arcs/circles
        special_vertices = []
        special_vertex_indices = {}
        selected_edges = []
        unselected_edges = []
        special_edges = []

        arc_groups = []
        circle_groups = []
        for i, group in enumerate(obj.vertex_groups):
            if "IFCARCINDEX" in group.name:
                arc_groups.append(i)
            elif "IFCCIRCLE" in group.name:
                circle_groups.append(i)

        arcs = {}
        circles = {}

        bm = bmesh.from_edit_mesh(obj.data)

        # https://docs.blender.org/api/blender_python_api_2_63_8/bmesh.html#CustomDataAccess
        # This is how we access vertex groups via bmesh, apparently, it's not very intuitive
        deform_layer = bm.verts.layers.deform.active

        for vertex in bm.verts:
            co = tuple(obj.matrix_world @ vertex.co)
            all_vertices.append(co)
            if vertex.hide:
                continue
            
            # TODO: iterate over deform layers instead of all vertex groups?
            # move to separate function `bm_check_vertex_in_groups`
            is_arc = False
            for group_index in arc_groups:
                if group_index in vertex[deform_layer]:
                    is_arc = True
                    break
            if is_arc:
                arcs.setdefault(group_index, []).append(vertex)
                special_vertex_indices[vertex.index] = group_index

            is_circle = False
            for group_index in circle_groups:
                if group_index in vertex[deform_layer]:
                    is_circle = True
                    break
            if is_circle:
                circles.setdefault(group_index, []).append(vertex)
                special_vertex_indices[vertex.index] = group_index

            if vertex.select:
                selected_vertices.append(co)
            else:
                if len(vertex.link_edges) > 1 and is_circle:
                    error_vertices.append(co)
                elif is_circle:
                    special_vertices.append(co)
                elif len(vertex.link_edges) != 2:
                    error_vertices.append(co)
                elif is_arc:
                    special_vertices.append(co)
                else:
                    unselected_vertices.append(co)

        for edge in bm.edges:
            edge_indices = [v.index for v in edge.verts]
            if edge.hide:
                continue
            if edge.select:
                selected_edges.append(edge_indices)
            else:
                i1, i2 = edge.verts[0].index, edge.verts[1].index
                # making sure that both vertices are in the same group
                if i1 in special_vertex_indices \
                    and special_vertex_indices[i1] == special_vertex_indices.get(i2, None):
                    special_edges.append(edge_indices)
                else:
                    unselected_edges.append(edge_indices)

        white = (1, 1, 1, 1)
        green = (0.545, 0.863, 0, 1)
        red = (1, 0.2, 0.322, 1)
        blue = (0.157, 0.565, 1, 1)
        grey = (0.2, 0.2, 0.2, 1)

        self.shader = gpu.shader.from_builtin("3D_UNIFORM_COLOR")

        def create_batch(shader_type, content_pos, color, indices=None, bind=True):
            batch = batch_for_shader(self.shader, shader_type, {"pos": content_pos}, indices=indices)
            # TODO: what's bind is for?
            if bind:
                self.shader.bind()
            self.shader.uniform_float("color", color)
            batch.draw(self.shader)

        # TODO: replace colors for tests
        create_batch("LINES", all_vertices, green, unselected_edges)
        create_batch("LINES", all_vertices, white, selected_edges)
        create_batch("LINES", all_vertices, grey, special_edges)
        create_batch("POINTS", unselected_vertices, green)
        create_batch("POINTS", error_vertices, red)
        create_batch("POINTS", special_vertices, blue)
        create_batch("POINTS", selected_vertices, white)

        # Draw arcs
        arc_centroids = []
        arc_segments = []
        for arc in arcs.values():
            if len(arc) != 3:
                continue
            sorted_arc = [None, None, None]
            for v1 in arc:
                connections = 0
                for link_edge in v1.link_edges:
                    v2 = link_edge.other_vert(v1)
                    if v2 in arc:
                        connections += 1
                if connections == 2:  # Midpoint
                    sorted_arc[1] = v1
                else:
                    sorted_arc[2 if sorted_arc[2] is None else 0] = v1
            points = [tuple(obj.matrix_world @ v.co) for v in sorted_arc]
            centroid = tool.Cad.get_center_of_arc(points)
            if centroid:
                arc_centroids.append(tuple(centroid))
            arc_segments.append(tool.Cad.create_arc_segments(pts=points, num_verts=17, make_edges=True))

        create_batch("POINTS", arc_centroids, grey, bind=False)
        for verts, edges in arc_segments:
            create_batch('LINES', verts, blue, edges, bind=False)

        # Draw circles
        circle_centroids = []
        circle_segments = []
        for circle in circles.values():
            if len(circle) != 2:
                continue
            p1 = obj.matrix_world @ circle[0].co
            p2 = obj.matrix_world @ circle[1].co
            radius = (p2 - p1).length / 2
            centroid = p1.lerp(p2, 0.5)
            circle_centroids.append(tuple(centroid))
            segments = self.create_circle_segments(360, 20, radius)
            matrix = obj.matrix_world.copy()
            matrix.col[3] = centroid.to_4d()
            segments = [[list(matrix @ Vector(v)) for v in segments[0]], segments[1]]
            circle_segments.append(segments)

        create_batch('POINTS', circle_centroids, grey, bind=False)
        for verts, edges in circle_segments:
            create_batch('LINES', verts, blue, edges, bind=False)

    def create_matrix(self, p, x, y, z):
        return Matrix([x, y, z, p]).to_4x4().transposed()

    # https://github.com/nortikin/sverchok/blob/master/nodes/generator/basic_3pt_arc.py
    # This function is taken from Sverchok, licensed under GPL v2-or-later.
    # This is a combination of the make_verts and make_edges function.
    def create_circle_segments(self, Angle, Vertices, Radius):
        if Angle < 360:
            theta = Angle / (Vertices - 1)
        else:
            theta = Angle / Vertices
        listVertX = []
        listVertY = []
        for i in range(Vertices):
            listVertX.append(Radius * cos(radians(theta * i)))
            listVertY.append(Radius * sin(radians(theta * i)))

        if Angle < 360 and self.mode_ == 0:
            sigma = radians(Angle)
            listVertX[-1] = Radius * cos(sigma)
            listVertY[-1] = Radius * sin(sigma)
        elif Angle < 360 and self.mode_ == 1:
            listVertX.append(0.0)
            listVertY.append(0.0)

        points = list((x, y, 0) for x, y in zip(listVertX, listVertY))

        listEdg = [(i, i + 1) for i in range(Vertices - 1)]

        if Angle < 360 and self.mode_ == 1:
            listEdg.append((0, Vertices))
            listEdg.append((Vertices - 1, Vertices))
        else:
            listEdg.append((Vertices - 1, 0))

        return points, listEdg
