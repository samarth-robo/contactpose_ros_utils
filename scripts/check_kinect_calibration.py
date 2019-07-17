import open3d as o3d
import os
import numpy as np
import transforms3d.quaternions as txq

osp = os.path
o3dio = o3d.io
o3dv = o3d.visualization


def show(input_dir):
  input_dir = osp.expanduser(input_dir)

  pcd_filename = osp.join(input_dir, '00000.pcd')
  pc = o3dio.read_point_cloud(pcd_filename)

  pose_filename = osp.join(input_dir, '00000_pose.txt')
  p = np.loadtxt(pose_filename, delimiter=',')
  trans, q = p[:3], p[3:]
  q /= np.linalg.norm(q)

  cTo = np.eye(4)
  cTo[:3,  3] = trans
  cTo[:3, :3] = txq.quat2mat([q[3], q[0], q[1], q[2]])

  mesh_filename = osp.join('~', 'dropbox', 'contactdb_v2', 'data',
                           'stl_files_notched', 'camera.stl')
  mesh = o3dio.read_triangle_mesh(osp.expanduser(mesh_filename))
  mesh.compute_vertex_normals()
  mesh.scale(1e-3, False)
  mesh.transform(cTo)

  o3dv.draw_geometries([mesh, pc])


if __name__ == '__main__':
  input_dir = osp.join('~', 'contactdb_data', 'test', 'camera')
  show(input_dir)