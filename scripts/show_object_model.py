# shows the object model with marker locations and recesses
import os
import open3d as o3d
import numpy as np

osp = os.path
o3dg = o3d.geometry
o3du = o3d.utility
o3dv = o3d.visualization
o3dio = o3d.io


def show(object_name, data_dir):
  mesh_filename = osp.join(data_dir, '{:s}.stl'.format(object_name))
  mesh = o3dio.read_triangle_mesh(mesh_filename)
  mesh.compute_vertex_normals()
  mesh.paint_uniform_color([0, 1, 1])
  v = np.asarray(mesh.vertices)

  oXs = np.loadtxt(osp.join(data_dir,
    '{:s}_all_marker_locations.txt'.format(object_name)))
  oXs *= 1000
  markers = []
  for oX in oXs:
    m = o3dg.TriangleMesh.create_sphere(radius=0.5)
    m.compute_vertex_normals()
    m.paint_uniform_color([1, 0, 0])
    m.translate(oX)
    markers.append(m)

  o3dv.draw_geometries([mesh] + markers)


if __name__ == '__main__':
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--object_name', required=True)
  parser.add_argument('--data_dir', default=osp.join('data', 'contactdb_data',
    'stl_files_notched'))
  args = parser.parse_args()

  show(args.object_name, args.data_dir)
