import numpy as np
import open3d as o3d
from lxml import etree
import os
import argparse
from IPython.core.debugger import set_trace

osp = os.path
o3dg = o3d.geometry
o3du = o3d.utility
o3dv = o3d.visualization
o3dio = o3d.io


def consolidate_marker_points(object_name, picked_points_dir,
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
  optX = np.asarray(o_pts)

  # read picked points
  filename = osp.join(picked_points_dir,
                      '{:s}_marker_locations.txt'.format(object_name))
  objX = np.loadtxt(filename, delimiter=',')[:, :3]
  objX /= 1000.0

  # read alignment
  filename = osp.join(rigid_bodies_dir,
                      'optitrack_{:s}_alignment.txt'.format(object_name))
  optTobj = np.loadtxt(filename)
  objTopt = np.linalg.inv(optTobj)

  optX = np.vstack((optX.T, np.ones(len(optX))))
  t_objX = objTopt @ optX
  t_objX = t_objX[:3].T
  filename = osp.join(picked_points_dir,
                      '{:s}_all_marker_locations.txt'.format(object_name))
  np.savetxt(filename, t_objX)
  print('{:s} written'.format(filename))

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--object_name', required=True)
  parser.add_argument('--picked_points_dir', required=True)
  args = parser.parse_args()
  consolidate_marker_points(args.object_name, args.picked_points_dir)
