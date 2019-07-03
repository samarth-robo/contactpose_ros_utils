import open3d
import os 
import argparse
import numpy as np
from copy import deepcopy
from embed_marker_points import rotmat_from_vecs, create_intruder
from IPython.core.debugger import set_trace

osp = os.path
o3dg = open3d.geometry
o3du = open3d.utility
o3dv = open3d.visualization


def check(mesh_filename, marker_diameter_mm):
  mesh_filename = osp.expanduser(mesh_filename)
  mesh_o3d = open3d.io.read_triangle_mesh(mesh_filename)
  mesh_o3d.compute_vertex_normals()

  # create intruder
  cyl = create_intruder(marker_diameter_mm)

  # read points
  points_filename = mesh_filename.replace('.stl', '_marker_locations.txt')
  d = np.loadtxt(points_filename, delimiter=',')
  if d.ndim == 1:
    d = d[np.newaxis, :]
  pts, normals = d[:, :3], d[:, 3:]

  # transform intruders to their appropriate locations on the object surface
  intruders_o3d = []
  for pt, normal in zip(pts, normals):
    T = rotmat_from_vecs(-normal, [0, 0, 1])
    T[:3,  3] = pt
    intruder_o3d = deepcopy(cyl)
    intruder_o3d.transform(T)
    intruder_o3d.paint_uniform_color(np.random.rand(3, 1))
    intruders_o3d.append(intruder_o3d)

  o3dv.draw_geometries(intruders_o3d + [mesh_o3d])


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--input_filename', required=True)
  parser.add_argument('--marker_diameter_mm', type=float, default=3,
      help="Diameter of marker as ordered")
  args = parser.parse_args()
  check(args.input_filename, args.marker_diameter_mm)
