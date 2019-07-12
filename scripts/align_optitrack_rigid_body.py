import numpy as np
import open3d as o3d
from lxml import etree
import os
import argparse
from scripts.global_pointcloud_registration import register

osp = os.path
o3dg = o3d.geometry
o3du = o3d.utility
o3dv = o3d.visualization


def align_optitrack_rigid_body(object_name, picked_points_dir,
    rigid_bodies_dir=osp.join('..', 'data', 'contactdb_data',
                              'optitrack_rigid_bodies')):
  # read Optitrack rigid body
  filename = osp.join(rigid_bodies_dir, '{:s}.motive'.format(object_name))
  data = etree.parse(filename).getroot()
  data = data[0][0]
  o_pts = []
  o_ids = []
  for markers in data:
    if markers.tag != 'markers':
      continue
    for marker in markers:
      for child in marker:
        if child.tag == 'position':
          p = child.text.split(',')
          o_pts.append([float(p[0]), float(p[1]), float(p[2])])
        elif child.tag == 'label_id':
          o_ids.append(int(child.text))
  o_pc = o3dg.PointCloud()
  o_pc.points = o3du.Vector3dVector(np.asarray(o_pts) * 1000)

  # read picked points
  filename = osp.join(picked_points_dir,
                      '{:s}_marker_locations.txt'.format(object_name))
  d = np.loadtxt(filename, delimiter=',')
  p_pts = d[:, :3]
  p_pc = o3dg.PointCloud()
  p_pc.points = o3du.Vector3dVector(np.asarray(p_pts))

  assert len(o_pc.points) == len(p_pc.points)
  T = register(p_pc, o_pc)
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
  args = parser.parse_args()
  align_optitrack_rigid_body(args.object_name, args.picked_points_dir)
