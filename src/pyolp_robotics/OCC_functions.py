from OCC.Core.BRep import BRep_Tool, BRep_Tool_Triangulation, BRep_Builder
from OCC.Core.BRepBuilderAPI import (
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_Transform,
    BRepBuilderAPI_MakeEdge,
    BRepBuilderAPI_MakeVertex,
)
from OCC.Core.BRepOffsetAPI import BRepOffsetAPI_MakePipe
from OCC.Core.BRepAlgoAPI import (
    BRepAlgoAPI_Fuse,
    BRepAlgoAPI_Common,
    BRepAlgoAPI_Cut,
    BRepAlgoAPI_Section,
)
from OCC.Core.BRepAdaptor import BRepAdaptor_CompCurve
from OCC.Core.BRepBuilderAPI import (
    BRepBuilderAPI_Transform,
    BRepBuilderAPI_MakeEdge,
    BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_MakeFace,
)
from OCC.Core.BRepOffsetAPI import BRepOffsetAPI_MakePipe
from OCC.Core.BRepPrimAPI import (
    BRepPrimAPI_MakeBox,
    BRepPrimAPI_MakeSphere,
    BRepPrimAPI_MakeCylinder,
    BRepPrimAPI_MakeTorus,
)
from OCC.Core.GCPnts import GCPnts_UniformAbscissa
from OCC.Core.Geom import Geom_Circle
from OCC.Core.GeomAPI import GeomAPI_PointsToBSpline
from OCC.Core.gp import (
    gp_Vec,
    gp_Ax2,
    gp_Pnt,
    gp_Dir,
    gp_Pln,
    gp_Trsf,
    gp_XYZ,
    gp_Circ,
    gp_Elips,
    gp_Ax1,
    gp_Ax3,
    gp_Quaternion,
    gp_DX,
    gp_DY,
    gp_DZ,
    gp_OZ,
    gp_EulerSequence,
)
from OCC.Core.TColgp  import TColgp_Array1OfPnt
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.TopAbs import (
    TopAbs_EDGE,
    TopAbs_FACE,
    TopAbs_SHELL,
    TopAbs_VERTEX,
    TopAbs_WIRE,
    TopAbs_SOLID,
    TopAbs_COMPOUND,
    TopAbs_COMPSOLID,
    TopAbs_IN
)
from OCC.Core.TopoDS import (
    topods_Edge,
    TopoDS_Edge,
    topods_Face,
    topods_Shell,
    topods_Vertex,
    topods_Wire,
    TopoDS_Wire,
    topods_Solid,
    topods_Compound,
    TopoDS_Compound,
    topods_CompSolid,
    topods_Edge,
    topods_Face,
    TopoDS_Iterator,
    TopoDS_Shape,
    TopoDS_Shell,
)

from OCC.Extend.ShapeFactory import *
from OCC.Extend.DataExchange import read_step_file, write_step_file, read_stl_file, write_stl_file
from OCC.Extend.LayerManager import Layer

from OCCUtils.Common import filter_points_by_distance, curve_length
from OCCUtils.edge import Edge
from OCCUtils.Topology import Topo

import collections
import fcl
import time
from itertools import combinations
import numpy as np
import math

def create_vec_2pts(from_pnt, to_pnt):
    v = gp_Vec()
    x = to_pnt.X() - from_pnt.X()
    y = to_pnt.Y() - from_pnt.Y()
    z = to_pnt.Z() - from_pnt.Z()
    v.SetCoord(x, y, z)
    return v

def create_dir_2pts(from_pnt, to_pnt):
    v = gp_Vec()
    x = to_pnt.X() - from_pnt.X()
    y = to_pnt.Y() - from_pnt.Y()
    z = to_pnt.Z() - from_pnt.Z()
    v.SetCoord(x, y, z)
    d = gp_Dir(v)
    return d

def dir_rotation(shape, point, dir_, angle):
    r'''
    angle in radians
    '''
    if type(point) is gp_XYZ:
        point = gp_Pnt(point)
    axis = gp_Ax1(point, dir_)
    if type(shape) is gp_Pnt:
        point_rot = shape.Rotated(axis, angle)
        # point_rot = shape.Rotated(axis, angle)
        return point_rot
    elif type(shape) is gp_Ax3:
        point_rot = shape.Rotated(axis, angle)
        # point_rot = shape.Rotated(axis, angle)
        return point_rot
    else:
        trns = gp_Trsf()
        trns.SetRotation(axis, angle)
        # trns.SetRotation(axis,angle)
        brep_trns = BRepBuilderAPI_Transform(shape, trns, True)
        brep_trns.Build()
        shape_rot = brep_trns.Shape()
        return shape_rot

def offset_with_dir_ax3(ax3=gp_Ax3(), dir_=gp_Dir(), offset_value=0):
    if offset_value == 0:
        offset_value = 0.01
    point = ax3.Location()
    normale = dir_
    x = normale.X() * (0 + offset_value)
    y = normale.Y() * (0 + offset_value)
    z = normale.Z() * (0 + offset_value)
    normale_dist = gp_Vec(x, y, z)
    trns = gp_Trsf()
    trns.SetTranslation(normale_dist)
    gp_Pnt()
    point.Transformed(trns)
    ax3_offset = ax3.Transformed(trns)
    return ax3_offset

def create_compound(shapes):
    compound = TopoDS_Compound()
    B = BRep_Builder()
    B.MakeCompound(compound)
    # Populate the compound
    # print('populating')
    # print(invalid_index)
    for shape in shapes:
        B.Add(compound, shape)
    return compound

def Move2pts(shapes, from_pnt, to_pnt):
    """
    Move 2 points_________-
    o_from pnt____________-
    o_to pnt______________-
    """
    vectors = []
    result = []
    if type(shapes) is list:
        v = gp_Vec()
        x = to_pnt.X() - from_pnt.X()
        y = to_pnt.Y() - from_pnt.Y()
        z = to_pnt.Z() - from_pnt.Z()
        v.SetCoord(x, y, z)
        trns = gp_Trsf()
        trns.SetTranslation(v)
        pts_moved = []
        for shape in shapes:
            if type(shape) is gp_Pnt:
                pt_moved = gp_Pnt()
                pt_moved = shape.Transformed(trns)
                # display.DisplayShape(pt_moved)
                pts_moved.append(pt_moved)
            else :
                translated = BRepBuilderAPI_Transform(shape, trns).Shape()
                pts_moved.append(translated)
        return pts_moved
    else:
        if type(from_pnt) is list:
            for f, t in zip(from_pnt, to_pnt):
                v = gp_Vec()
                x = t.X() - f.X()
                y = t.Y() - f.Y()
                z = t.Z() - f.Z()
                v.SetCoord(x, y, z)
                vectors.append(v)
            for (
                sh,
                v,
            ) in zip(shapes, vectors):
                trns = gp_Trsf()
                trns.SetTranslation(v.Reversed())
                translated = BRepBuilderAPI_Transform(sh, trns).Shape()
                result.append(translated)
            return result
        else:
            if type(shapes) is gp_Pnt:
                v = gp_Vec()
                x = to_pnt.X() - from_pnt.X()
                y = to_pnt.Y() - from_pnt.Y()
                z = to_pnt.Z() - from_pnt.Z()
                v.SetCoord(x, y, z)
                trns = gp_Trsf()
                trns.SetTranslation(v)
                pt_moved = gp_Pnt()
                pt_moved = shapes.Transformed(trns)
                return pt_moved
            else:
                v = gp_Vec()
                x = to_pnt.X() - from_pnt.X()
                y = to_pnt.Y() - from_pnt.Y()
                z = to_pnt.Z() - from_pnt.Z()
                v.SetCoord(x, y, z)
                trns = gp_Trsf()
                trns.SetTranslation(v)
                translated = BRepBuilderAPI_Transform(shapes, trns).Shape()
                return translated


def rotate_shape(shape, axis, angle, unite="deg"):
    """Rotate a shape around an axis, with a given angle.
    @param shape : the shape to rotate
    @point : the origin of the axis
    @vector : the axis direction
    @angle : the value of the rotation
    @return: the rotated shape.
    """
    if unite == "deg":  # convert angle to radians
        angle = math.radians(angle)
    trns = gp_Trsf()
    trns.SetRotation(axis, angle)
    if type(shape) is gp_Pnt():
        print("point")
    else:
        brep_trns = BRepBuilderAPI_Transform(shape, trns, False)
        brep_trns.Build()
        shp = brep_trns.Shape()
    return shp

def create_compound_from_points(list_pts):
    compound = TopoDS_Compound()
    B = BRep_Builder()
    B.MakeCompound(compound)
    # Populate the compound
    for i, el in enumerate(list_pts):
        if type(el) is list:
            for i, e in enumerate(el):
                ev = BRepBuilderAPI_MakeVertex(e).Shape()
                B.Add(compound, ev)
        else:
            elv = BRepBuilderAPI_MakeVertex(el).Shape()
            B.Add(compound, elv)
    return compound

def MidPoint(pointA, pointB):
    """
    MidPoint_____________-
    o_Point A____________-
    o_Point B______________-
    """
    vec1 = gp_Vec(pointA.XYZ())
    vec2 = gp_Vec(pointB.XYZ())
    midvec = (vec1 + vec2) / 2.0
    midpoint = gp_Pnt(midvec.XYZ())
    return midpoint

def get_dir_from_edge(edge):
    """
    Dir from Edge________-
    o_Edge_______________-
    """
    edg = Edge(edge)
    first_point = BRep_Tool.Pnt(edg.first_vertex())
    last_point = BRep_Tool.Pnt(edg.last_vertex())
    dir_edge = gp_Dir(
        last_point.X() - first_point.X(),
        last_point.Y() - first_point.Y(),
        last_point.Z() - first_point.Z(),
    )
    return dir_edge

def TwoPtsEdge(pnt1, pnt2):
    """
    Generates 2 pts Edge__-
    o_Point_______________-
    o_Point_______________-
    """
    if type(pnt1) is list:
        edges = []
        for p1, p2 in zip(pnt1, pnt2):
            edge = BRepBuilderAPI_MakeEdge(p1, p2).Edge()
            edges.append(edge)
        return edges
    else:
        edge = BRepBuilderAPI_MakeEdge(pnt1, pnt2).Edge()
        return edge
    
def create_edge(pnt=gp_Pnt(), dir=gp_Dir(), length=100):
    start_pnt = pnt
    vector = gp_Vec(dir)
    end_pnt = start_pnt.Translated(vector.Scaled(length))
    edge = TwoPtsEdge(start_pnt, end_pnt)
    return edge

def Wire(pointslist, closed=False):
    """
    Generates Wire________-
    o_List of Points______-
    """
    pointsarray = TColgp_Array1OfPnt(1, len(pointslist))
    for n, i in enumerate(pointslist):
        pointsarray.SetValue(n + 1, i)
    wirebuild = BRepBuilderAPI_MakeWire()
    newpoint = None
    for i in range(1, len(pointslist)):
        newpoint = pointsarray.Value(i + 1)
        if pointsarray.Value(i) == newpoint:
            pass
        else:
            edgepoint = BRepBuilderAPI_MakeEdge(
                pointsarray.Value(i), pointsarray.Value(i + 1)
            ).Edge()
            wirebuild.Add(edgepoint)
    if closed:
        last_edge = BRepBuilderAPI_MakeEdge(
            pointsarray.Value(len(pointslist)), pointsarray.Value(1)
        ).Edge()
        wirebuild.Add(last_edge)
        # return wirebuild.Shape()
    return wirebuild.Shape()

def DiscretizeWire(wire, nbpts):
    """
    Discretize Wire_______-
    o_Wire________________-
    o_Nb of points________-
    """
    pnts = []  # points to create bsplines
    if isinstance(wire, TopoDS_Edge):
        wire = BRepBuilderAPI_MakeWire(wire).Wire()
    curve_adapt = BRepAdaptor_CompCurve(wire)
    # print(curve_adapt)
    _lbound, _ubound = curve_adapt.FirstParameter(), curve_adapt.LastParameter()
    npts = GCPnts_UniformAbscissa(curve_adapt, nbpts, _lbound, _ubound)
    if npts.IsDone():
        for i in range(1, npts.NbPoints() + 1):
            pnts.append(curve_adapt.Value(npts.Parameter(i)))
    return pnts

def CurveLength(curves):
    """
    Curve Length__________-
    o_Wire/Edge(L)________-
    """
    lengths = []
    for curve in curves:
        lengths.append(curve_length(curve))
    return lengths
    # print(tmp)

def Pipe(wire, radius):
    """
    Generates pipe________-
    o_Wire________________-
    o_Radius______________-
    """
    if isinstance(wire, TopoDS_Edge):
        wire = BRepBuilderAPI_MakeWire(wire).Wire()
    vertices = DiscretizeWire(wire, 100)
    dir_ = gp_Dir(
        vertices[1].X() - vertices[0].X(),
        vertices[1].Y() - vertices[0].Y(),
        vertices[1].Z() - vertices[0].Z(),
    )
    if radius == 0:
        radius = 0.01
    circle = gp_Circ(gp_Ax2(vertices[0], dir_), radius)
    profile_edge = BRepBuilderAPI_MakeEdge(circle).Edge()
    profile_wire = BRepBuilderAPI_MakeWire(profile_edge).Wire()
    profile_face = BRepBuilderAPI_MakeFace(profile_wire).Face()
    pipe = BRepOffsetAPI_MakePipe(wire, profile_face).Shape()
    return pipe

def Circle(axis, radius):
    """
    Draw circle______________-
    o_Ax2____________________-
    o_Radius_________________-
    """
    circle = Geom_Circle(axis, radius)
    return circle

def Bspline(list_of_pnts):
    length_array = len(list_of_pnts)
    # the bspline profile
    array = TColgp_Array1OfPnt(1, length_array)
    for i, pnt in enumerate(list_of_pnts, start=1):
        array.SetValue(i, pnt)
    bspline = GeomAPI_PointsToBSpline(array, 3, 8).Curve()
    profile = BRepBuilderAPI_MakeEdge(bspline).Edge()
    return profile

def Box(width, length, height):
    """
    Generates box_________-
    o_Width_______________-
    o_Length______________-
    o_Height______________-
    """
    box = BRepPrimAPI_MakeBox(gp_Pnt(), width, length, height).Shape()
    return box

def Sphere(point, radius):
    """
    Generates sphere_________-
    o_Center point/ax2_______-
    o_Radius_________________-
    """
    sphere = BRepPrimAPI_MakeSphere(point, radius).Shape()
    return sphere

def Cylinder(axe, radius, length):
    """
    Generates cylinder_______-
    o_Axe____________________-
    o_Radius_________________-
    o_Length_________________-
    """
    cylinder = BRepPrimAPI_MakeCylinder(axe, radius, length).Shape()
    return cylinder

def Torus(axe, distance, radius):
    """
    Generates torus__________-
    o_Ax2____________________-
    o_Distance center/center_-
    o_Radius_________________-
    """
    torus = BRepPrimAPI_MakeTorus(axe, distance, radius).Shape()
    return torus

def Fuse(a, b):
    """
    Generates fusion_________-
    o_Part 1 (or list)_______-
    o_Part 2_________________-
    """
    if type(a) is list:
        count = len(a)
        fuse_shps = {}
        ijk = 0
        fuse_shps[ijk] = BRepAlgoAPI_Fuse(a[0], a[1]).Shape()
        for i in range(2, count):
            ijk += 1
            fuse_shps[ijk] = BRepAlgoAPI_Fuse(fuse_shps[ijk - 1], a[i]).Shape()
        return fuse_shps[ijk]
    else:
        fuse_shp = BRepAlgoAPI_Fuse(a, b).Shape()
        return fuse_shp

def Common(a, b):
    """
    Generates common_________-
    o_Part 1_________________-
    o_Part 2_________________-
    """
    common_shp = BRepAlgoAPI_Common(a, b).Shape()
    return common_shp

def Cut(basis, cutter):
    """
    Generates cutting________-
    o_Basis__________________-
    o_Cutter (or list)_______-
    """
    if type(cutter) is list and type(basis) is not list:
        count = len(cutter)
        cut_shps = {}
        ijk = 0
        cut_shps[ijk] = BRepAlgoAPI_Cut(basis, cutter[0]).Shape()
        for i in range(1, count):
            ijk += 1
            cut_shps[ijk] = BRepAlgoAPI_Cut(cut_shps[ijk - 1], cutter[i]).Shape()
        return cut_shps[ijk]
    elif type(basis) is list and type(cutter) is not list:
        cut_shps = []
        for b in basis:
            cut_shps.append(BRepAlgoAPI_Cut(b, cutter).Shape())
        return cut_shps
    else:
        cut_shp = BRepAlgoAPI_Cut(basis, cutter).Shape()
        return cut_shp

def Section(basis, cutter):
    """
    Generates Sections_______-
    o_Basis__________________-
    o_Cutter (or list)_______-
    """

    if type(cutter) is list and type(basis) is not list:
        count = len(cutter)
        cut_shps = {}
        ijk = 0
        cut_shps[ijk] = BRepAlgoAPI_Section(basis, cutter[0]).Shape()
        for i in range(1, count):
            ijk += 1
            cut_shps[ijk] = BRepAlgoAPI_Section(cut_shps[ijk - 1], cutter[i]).Shape()
        return cut_shps[ijk]
    elif type(basis) is list and type(cutter) is not list:
        cut_shps = []
        for b in basis:
            cut_shps.append(BRepAlgoAPI_Section(b, cutter).Shape())
        return cut_shps
    else:
        cut_shp = BRepAlgoAPI_Section(basis, cutter).Shape()
        return cut_shp

def TopExplorer(shape, to_find=None, filter_pts=0.01):
    """
    Topology Explorer________-
    o_Shape__________________-
    """
    # find vertices
    if to_find == "vertex":
        topexp_vertex = TopExp_Explorer()
        topexp_vertex.Init(shape, TopAbs_VERTEX)
        vertices = []
        while topexp_vertex.More():
            vert = topods_Vertex(topexp_vertex.Current())
            point = BRep_Tool.Pnt(vert)
            vertices.append(point)
            topexp_vertex.Next()
        vertices_red = filter_points_by_distance(vertices, filter_pts)
        return vertices_red

    # find edges
    elif to_find == "edge":
        topexp_edge = TopExp_Explorer()
        topexp_edge.Init(shape, TopAbs_EDGE)
        edges = []
        while topexp_edge.More():
            edge = topods_Edge(topexp_edge.Current())
            edges.append(edge)
            topexp_edge.Next()
        return edges

    # find wires
    elif to_find == "wire":
        topexp_wire = TopExp_Explorer()
        topexp_wire.Init(shape, TopAbs_WIRE)
        wires = []
        while topexp_wire.More():
            wire = topods_Wire(topexp_wire.Current())
            wires.append(wire)
            topexp_wire.Next()
        return wires

    # find faces
    elif to_find == "face":
        topexp_face = TopExp_Explorer()
        topexp_face.Init(shape, TopAbs_FACE)
        faces = []
        while topexp_face.More():
            face = topods_Face(topexp_face.Current())
            faces.append(face)
            topexp_face.Next()
        return faces

    # find shells
    elif to_find == "shell":
        topexp_shell = TopExp_Explorer()
        topexp_shell.Init(shape, TopAbs_SHELL)
        shells = []
        while topexp_shell.More():
            shell = topods_Shell(topexp_shell.Current())
            shells.append(shell)
            topexp_shell.Next()
        return shells

    # find solids
    elif to_find == "solid":
        topexp_solid = TopExp_Explorer()
        topexp_solid.Init(shape, TopAbs_SOLID)
        solids = []
        while topexp_solid.More():
            solid = topods_Solid(topexp_solid.Current())
            solids.append(solid)
            topexp_solid.Next()
        return solids

    # find compounds
    elif to_find == "compound":
        topexp_compound = TopExp_Explorer()
        topexp_compound.Init(shape, TopAbs_COMPOUND)
        compounds = []
        while topexp_compound.More():
            compound = topods_Compound(topexp_compound.Current())
            compounds.append(compound)
            topexp_compound.Next()
        return compounds

    # find compsolids
    elif to_find == "composolid":
        topexp_compsolid = TopExp_Explorer()
        topexp_compsolid.Init(shape, TopAbs_COMPSOLID)
        compsolids = []
        while topexp_compsolid.More():
            compsolid = topods_CompSolid(topexp_compsolid.Current())
            compsolids.append(compsolid)
            topexp_compsolid.Next()
        return compsolids

    else:
        return "please specify your search"


def distance_two_points(p1, p2):
    distance = math.sqrt(
        math.pow(p2.X() - p1.X(), 2)
        + math.pow(p2.Y() - p1.Y(), 2)
        + math.pow(p2.Z() - p1.Z(), 2)
    )
    return distance

def offset_ax3(ax3=gp_Ax3(), offset_value=0):
    if offset_value == 0:
        offset_value = 0.01
    point = ax3.Location()
    normale = ax3.Direction()
    x = normale.X() * (0 + offset_value)
    y = normale.Y() * (0 + offset_value)
    z = normale.Z() * (0 + offset_value)
    normale_dist = gp_Vec(x, y, z)
    trns = gp_Trsf()
    trns.SetTranslation(normale_dist)
    pt_to_go = gp_Pnt()
    pt_to_go = point.Transformed(trns)
    ax3 = gp_Ax3(pt_to_go, normale)
    return ax3

def get_euler_angles(ax3_1, ax3_2, degres=True):
    # transformation calculation from ax3_1 to ax3_2
    trsf = gp_Trsf()
    trsf.SetTransformation(ax3_1, ax3_2)

    # extract euler angles d'euler from the rotation matrix transformation
    rotation_matrix = trsf.GetRotation()
    print(rotation_matrix)
    gp_Extrinsic_XYZ = gp_EulerSequence(2)
    angles = rotation_matrix.GetEulerAngles(gp_Extrinsic_XYZ)
    angles_rad = [*angles]
    if degres :
        # conversion in degrees
        angles_degrees = [math.degrees(angle) for angle in angles]

        print("Euler angles in degrees from ax3_1 to ax3_2 : ", angles_degrees)
        return angles_degrees
    else:
        print("Euler angles in radians from ax3_1 to ax3_2 : ", angles_rad)
        return angles_rad
    
def get_quaternions(ax1, ax2):
    # transformation calculation from ax3_1 to ax3_2
    trsf = gp_Trsf()
    trsf.SetTransformation(ax1, ax2)

    # extract rotation matrix transformation
    rotation_matrix = trsf.GetRotation()
    quaternion_coord = (rotation_matrix.W(), rotation_matrix.X(), rotation_matrix.Y(), rotation_matrix.Z())
    return quaternion_coord

def get_trsf_2ax3(ax1, ax2):
    # transformation calculation from ax3_1 to ax3_2
    trsf = gp_Trsf()
    trsf.SetTransformation(ax1, ax2)
    return trsf

def trsf(shapes, trsf):
    """
    Generates transform___-
    o_[Shapes]____________-
    o_[trsf]___________-
    """
    if type(shapes) is list:
        result = []
        for sh in shapes:
            transformed = BRepBuilderAPI_Transform(sh, trsf).Shape()
            result.append(transformed)
        return result
    else:
        if type(shapes) is gp_Ax3:
            transformed = shapes.Transformed(trsf)
        else:
            transformed = BRepBuilderAPI_Transform(shapes, trsf).Shape()
        return transformed

def create_vec_2pts(from_pnt, to_pnt):
    v = gp_Vec()
    x = to_pnt.X() - from_pnt.X()
    y = to_pnt.Y() - from_pnt.Y()
    z = to_pnt.Z() - from_pnt.Z()
    v.SetCoord(x, y, z)
    return v

def slice_shape(shape):
    # Param
    Zmin, Zmax, deltaZ = -100, 100, 5
    if shape is None:
        # Create the shape to slice
        shape = BRepPrimAPI_MakeSphere(60.0).Shape()
    # Define the direction
    D = gp_Dir(0.0, 0.0, 1.0)  # the z direction
    # Perform slice
    sections = []
    init_time = time.time()  # for total time computation
    for z in range(Zmin, Zmax, deltaZ):
        # Create Plane defined by a point and the perpendicular direction
        P = gp_Pnt(0, 0, z)
        Pln = gp_Pln(P, D)
        face = BRepBuilderAPI_MakeFace(Pln).Shape()
        # Computes Shape/Plane intersection
        section_shp = BRepAlgoAPI_Section(shape, face)
        if section_shp.IsDone():
            sections.append(section_shp.Shape())
    total_time = time.time() - init_time
    print("%.3fs necessary to perform slice." % total_time)
    compound = create_compound(sections)
    return compound


def mesh_from_brep(occ_brep, theLinDeflection=0.8):
    """
    returns a list of triangles that represent the mesh, represening the `occ_brep` BRep
    """
    # create / update a mesh
    # this is important when a mesh has not been display
    # in this case it has no mesh to iterate through
    inc_mesh = BRepMesh_IncrementalMesh(occ_brep, theLinDeflection) 
    assert inc_mesh.IsDone()

    tp = Topo(occ_brep, ignore_orientation=True)
    triangles = collections.deque()

    for f in tp.faces():
        loc = TopLoc_Location()
        triangulation = BRep_Tool_Triangulation(f, loc)
        if triangulation is None:
            continue

        facing = triangulation #.GetObject()
        tri = facing.Triangles()
        # nodes = facing.Nodes()

        for i in range(1, facing.NbTriangles() + 1):
            trian = tri.Value(i)
            index1, index2, index3 = trian.Get()
            tria=facing.Node(index1) #.Value(index1) #.Coord()
            trib=facing.Node(index2) #.Value(index2) #.Coord()
            tric=facing.Node(index3) #.Value(index3) #.Coord()
            triangles.append(
                [   [tria.X(),tria.Y(),tria.Z()],
                    [trib.X(),trib.Y(),trib.Z()],
                    [tric.X(),tric.Y(),tric.Z()],
                ]
    )
    return triangles

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
        if collision is True :
            shapes_colliding.append(basis)
            shapes_colliding.append(cutter)
            if stop_at_first :
                continue_searching = False
    return collision, shapes_colliding
