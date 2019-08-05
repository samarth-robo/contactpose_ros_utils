import numpy as np
import open3d as o3d
from lxml import etree
import os
import argparse
from global_pointcloud_registration import register

osp = os.path
o3dg = o3d.geometry
o3du = o3d.utility
o3dv = o3d.visualization
o3dio = o3d.io


def align_optitrack_rigid_body(object_name, n_markers, picked_points_dir,
    rigid_bodies_dir=osp.join('..', 'data', 'contactdb_data',
                              'optitrack_rigid_bodies')):
  # read Optitrack rigid body
  filename = osp.join(rigid_bodies_dir, 'contactdb_assets.motive')
  data = etree.parse(filename).getroot()
  data = data[0]
  o_pts = []
  for rigid_body in data:
    for properties in rigid_body:
      if properties.tag != 'properties':
        continue
      if properties[0][1].text != object_name:
        break
      for marker in properties:
        if 'MarkerLocation' not in marker[0].text:
          continue
        p = marker[1].text.split(',')
        o_pts.append([float(p[0]), float(p[1]), float(p[2])])
  if len(o_pts) == 0:
    print('{:s} does not have {:s}'.format(filename, object_name))
    return
  o_pc = o3dg.PointCloud()
  o_pc.points = o3du.Vector3dVector(np.asarray(o_pts) * 1000)

  # read picked points
  filename = osp.join(picked_points_dir,
                      '{:s}_marker_locations.txt'.format(object_name))
  d = np.loadtxt(filename, delimiter=',')
  p_pts = d[:, :3]
  p_pc = o3dg.PointCloud()
  p_pc.points = o3du.Vector3dVector(np.asarray(p_pts))

  mesh_filename = osp.join(picked_points_dir, '..', 'stl_files_notched',
    '{:s}.stl'.format(object_name))
  source_mesh = o3dio.read_triangle_mesh(mesh_filename)

  T = register(p_pc, o_pc, source_mesh, n_markers)
  # convert to meters
  T[:3, 3] /= 1000.0
  filename = osp.join(rigid_bodies_dir,
                      'optitrack_{:s}_alignment.txt'.format(object_name))
  np.savetxt(filename, T)
  print('{:s} written.'.format(filename))

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--object_name', required=True)
  parser.add_argument('--picked_points_dir', required=True)
  parser.add_argument('--n_markers', type=int, required=True)
  args = parser.parse_args()
  align_optitrack_rigid_body(args.object_name, args.n_markers,
    args.picked_points_dir)
