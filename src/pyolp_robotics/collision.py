from itertools import combinations

import fcl
import numpy as np
from OCC.Extend.LayerManager import Layer
from OCC.Core.AIS import AIS_Shape
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.BRepExtrema import BRepExtrema_ShapeProximity
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.TopoDS import TopoDS_Shape

from pyolp_robotics.OCC_functions import mesh_from_brep


## FCL based code

def print_collision_result(o1_name, o2_name, result):
    print("Collision between {} and {}:".format(o1_name, o2_name))
    print("-" * 30)
    print("Collision?: {}".format(result.is_collision))
    print("Number of contacts: {}".format(len(result.contacts)))
    print("")


def fcl_collision_object_from_shape(shape1, shape2):
    """
    create a fcl.BVHModel instance from the `shape` TopoDS_Shape
    """
    triangles = mesh_from_brep(shape1)
    triangles = np.array(triangles)
    # Create mesh geometry
    _mesh1 = fcl.BVHModel()
    n_tris = len(triangles)
    _mesh1.beginModel(n_tris, n_tris * 3)
    for tri in triangles:
        x, y, z = tri
        _mesh1.addTriangle(x, y, z)
    _mesh1.endModel()

    triangles2 = mesh_from_brep(shape2)
    triangles2 = np.array(triangles2)
    # Create mesh geometry
    _mesh2 = fcl.BVHModel()
    n_tris = len(triangles2)
    _mesh2.beginModel(n_tris, n_tris * 3)
    for tri in triangles2:
        x, y, z = tri
        _mesh2.addTriangle(x, y, z)
    _mesh2.endModel()

    req = fcl.CollisionRequest(enable_contact=True)
    res = fcl.CollisionResult()

    n_contacts = fcl.collide(
        fcl.CollisionObject(_mesh1, fcl.Transform()),
        fcl.CollisionObject(_mesh2, fcl.Transform()),
        req,
        res,
    )
    print_collision_result("_mesh1", "_mesh2", res)
    return res.is_collision


def fcl_collisions_collection_shapes(shapes, stop_at_first=True):
    continue_searching = True
    shapes_colliding = []
    for two_solids in combinations(shapes, 2):
        if continue_searching:
            basis = two_solids[0]
            cutter = two_solids[1]
            collision = fcl_collision_object_from_shape(basis, cutter)
        if collision is True:
            shapes_colliding.append(basis)
            shapes_colliding.append(cutter)
            if stop_at_first:
                continue_searching = False
    return collision, shapes_colliding


def check_two_part_collisions(part1, part2, display_ais, get_collider=False):
    layer_for_collision = Layer(display_ais)
    layer_for_collision.add_shape(part1)
    layer_for_collision.add_shape(part2)
    ais_shapes = [(key, value[1]) for key, value in layer_for_collision.element_to_display.items()]
    basis = ais_shapes[0][1]
    cutter = ais_shapes[1][1]
    collider = Ais_Collision(basis, cutter)
    does_collide = collider.check_collision()
    if get_collider:
        return does_collide, collider
    else:
        return does_collide

def check_collections_collisions(shapes, display_ais, stop_at_first=True):
    layer_for_collision = Layer(display_ais)
    for shape in shapes:
        layer_for_collision.add_shape(shape)
    ais_shapes = ais_shapes = [(key, value[1]) for key, value in layer_for_collision.element_to_display.items()]
    continue_searching = True
    collision = False
    shapes_colliding = []
    for two_solids in combinations(ais_shapes, 2):
        if continue_searching:
            basis = two_solids[0][1]
            print("basis =", basis)
            cutter = two_solids[1][1]
            collide = Ais_Collision(basis, cutter)
            does_collide = collide.check_collision()
            if does_collide:
                collision = True
                basis_shape = shapes[two_solids[0][0]]
                shapes_colliding.append(basis_shape)
                cutter_shape = shapes[two_solids[1][0]]
                shapes_colliding.append(cutter_shape)
                if stop_at_first:
                    continue_searching = False
    return collision, shapes_colliding



## OCC handling collisions

class Ais_Collision:
    def __init__(self,
                 ais_a: TopoDS_Shape,
                 ais_b: TopoDS_Shape,
                 get_collision_a=True,
                 get_collision_b=True,
                 defl=1e01,
                 tol=0.1
                 ):

        """Use AIS objects for interference judgment
        Parameters
        ----------
        `ais_a` : AIS_Shape
            Interference object A
        `ais_b` : AIS_Shape
            Interference object B
        `get_collision_a` : bool, optional
            Whether to return the faces where collisions occur in A, default value: True
        `get_collision_b` : bool, optional
            Whether to return the faces where collisions occur in B, default value: True
        Returns
        -------
        _type_
            The faces where collisions occurred (including results from A and B)
        """
        # # Obtain the TopoDS of the AIS object
        _shape_a = ais_a.Shape()
        _shape_b = ais_b.Shape()
        # Obtain the Trsf of the AIS object
        trsf_a = ais_a.Transformation()
        trsf_b = ais_b.Transformation()
        # Move the original shape
        self.shape_a = BRepBuilderAPI_Transform(_shape_a, trsf_a).Shape()
        self.shape_b = BRepBuilderAPI_Transform(_shape_b, trsf_b).Shape()
        self.defl = defl
        self.tol = tol
        self.get_collision_a = get_collision_a
        self.get_collision_b = get_collision_b
        # Generate mesh
        self._compute_triangulation()

    def _compute_triangulation(self):
        self.mesh_a = BRepMesh_IncrementalMesh(self.shape_a, self.defl)
        self.mesh_a.Perform()
        self.mesh_b = BRepMesh_IncrementalMesh(self.shape_b, self.defl)
        self.mesh_b.Perform()

    def check_collision(self):
        # Collision check
        self.proximity = BRepExtrema_ShapeProximity(self.shape_a, self.shape_b, self.tol)
        self.proximity.Perform()
        return self.proximity.IsDone()

    def get_collision_sets(self):
        if self.proximity.IsDone():
            # Get the collision part of shape_a
            if self.get_collision_a:
                overlaps1 = self.proximity.OverlapSubShapes1()
                face_indices1 = overlaps1.Keys()
                collision_face_a = []
                for ind in face_indices1:
                    face = self.proximity.GetSubShape1(ind)
                    collision_face_a.append(face)

            # Get the collision part of shape_b
            if self.get_collision_b:
                overlaps2 = self.proximity.OverlapSubShapes2()
                face_indices2 = overlaps2.Keys()
                collision_face_b = []
                for ind in face_indices2:
                    face = self.proximity.GetSubShape2(ind)
                    collision_face_b.append(face)
            # Return the faces where interference occurred
            if collision_face_a or collision_face_b:
                return [collision_face_a, collision_face_b]
        else:
            return []