import pymesh 
import open3d as o3d
import os 
import argparse
import numpy as np
from copy import deepcopy
import transforms3d as xforms

osp = os.path
o3dg = o3d.geometry
o3du = o3d.utility
o3dv = o3d.visualization


def rotmat_from_vecs(v1, v2=np.asarray([0, 0, 1])):
  """ 
  Returns a rotation matrix 1R2 that rotates v2 to v1
  :param v1: vector in frame 1
  :param v2: vector in frame 2
  :return:
  """
  v1 = v1 / np.linalg.norm(v1)
  v2 = v2 / np.linalg.norm(v2)
  v = np.cross(v2, v1) 
  vx = np.asarray([
    [0,    -v[2], +v[1], 0], 
    [+v[2], 0,    -v[0], 0], 
    [-v[1], +v[0], 0,    0], 
    [0,     0,     0,    0]])
  dotp = np.dot(v1, v2) 
  if abs(dotp + 1) < 1e-3:
    # find vector perpendicular to both v1 and v2
    v3 = np.cross(v1, v2)
    if np.linalg.norm(v3) < 1e-1:
      # HACK, assumes v2 = [0, 0, 1]
      v3 = np.asarray([0, 1, 0])
    # rotate around vector by 180 degrees
    T = np.eye(4)
    T[:3, :3] = xforms.axangles.axangle2mat(v3, np.pi)
    return T

  return np.eye(4) + vx + np.dot(vx, vx)/(1+dotp)


def pick_points(mesh, mesh_pymesh, object_name):
  """
  Function for inteactively picking points
  """
  print("")
  print("1) Pick points using [shift + left click]")
  print("   Press [shift + right click] to undo point picking")
  print("2) After picking points, press q for close the window")
  # sample points on the mesh surface to be picked from
  pcd = mesh.sample_points_poisson_disk(10000)
  pcd.estimate_normals()

  # create interactive visualizer
  vis = o3dv.VisualizerWithEditing()
  vis.create_window(object_name)
  # vis.add_geometry(mesh)
  vis.add_geometry(pcd)
  vis.run()  # user picks points
  vis.destroy_window()
  print("")
  pt_idxs = vis.get_picked_points()

  # get the picked points and normals of the faces closest to those points
  pts = np.asarray(pcd.points)[pt_idxs]
  _, face_idxs, _ = pymesh.distance_to_mesh(mesh_pymesh, pts)
  normals = np.asarray(mesh.triangle_normals)[face_idxs]
  # normals = np.asarray(pcd.normals)[pt_idxs]
  if normals.ndim == 1:
    normals = normals[np.newaxis, :]
  return pts, normals


def pymesh2o3d(input_mesh):
  """
  Convert a mesh from PyMesh to Open3D
  """
  vertices_array = input_mesh.vertices.copy()
  faces_array = input_mesh.faces.copy()
  
  mesh_o3d = o3dg.TriangleMesh()
  mesh_o3d.vertices = o3du.Vector3dVector(vertices_array)
  mesh_o3d.triangles = o3du.Vector3iVector(faces_array)
  mesh_o3d.compute_vertex_normals()
  mesh_o3d.compute_triangle_normals()
  
  return mesh_o3d


def create_intruder(marker_diameter_mm):
  cyl_height = 10
  marker_depth_mm = np.ceil(marker_diameter_mm / 2.0)
  # 0.5 mm added as tolerance for 3D printing
  cyl = o3dg.TriangleMesh.create_cylinder((marker_diameter_mm+0.5)/2.0, cyl_height)
  cyl.compute_vertex_normals()
  T = np.eye(4)
  T[2, 3] = -cyl_height/2.0 + marker_depth_mm
  cyl.transform(T)
  return cyl


def embed(input_filename, output_dir, marker_diameter_mm, recut):
  input_filename = osp.expanduser(input_filename)
  object_name, ext = input_filename.split('/')[-1].split('.')
  output_dir = osp.expanduser(output_dir)

  input_mesh = pymesh.load_mesh(input_filename)
  # convert mesh to Open3D 
  mesh_o3d = pymesh2o3d(input_mesh)

  # create intruder
  cyl = create_intruder(marker_diameter_mm)

  if not recut:
    # pick points
    pts, normals = pick_points(mesh_o3d, input_mesh, object_name)
  else:
    # read points
    points_filename = input_filename.split('/')[-1]
    points_filename = points_filename.replace('.stl', '_marker_locations.txt')
    points_filename = osp.join(output_dir, points_filename)
    d = np.loadtxt(points_filename, delimiter=',')
    if d.ndim == 1:
      d = d[np.newaxis, :]
    pts, normals = d[:, :3], d[:, 3:]

  # transform intruders to their appropriate locations on the object surface
  intruders_o3d = []
  output_mesh = pymesh.meshio.form_mesh(input_mesh.vertices, input_mesh.faces)
  for pt, normal in zip(pts, normals):
    T = rotmat_from_vecs(-normal, [0, 0, 1])
    T[:3,  3] = pt
    intruder_o3d = deepcopy(cyl)
    intruder_o3d.transform(T)
    intruder_o3d.paint_uniform_color(np.random.rand(3, 1))
    intruders_o3d.append(intruder_o3d)

    intruder_pymesh = pymesh.meshio.form_mesh(
        np.asarray(intruder_o3d.vertices),
        np.asarray(intruder_o3d.triangles))
    # perform boolean operation to remove the intersecting part of the intruder 
    # from the object mesh
    # change the engine in case it doesn't work
    output_mesh = pymesh.boolean(output_mesh, intruder_pymesh,
        operation='difference', engine='auto')

  output_mesh_o3d = pymesh2o3d(output_mesh)
  # o3dv.draw_geometries(intruders_o3d + [mesh_o3d])
  o3dv.draw_geometries([output_mesh_o3d])

  # save points and normals
  output_filename = osp.join(output_dir,
      '{:s}_marker_locations.txt'.format(object_name))
  header = 'px,py,pz,nx,ny,nz'
  np.savetxt(output_filename, np.hstack((pts, normals)), delimiter=',',
      header=header)
  print('{:s} written'.format(output_filename))

  output_filename = osp.join(output_dir,
      '{:s}.{:s}'.format(object_name, ext))
  o3d.io.write_triangle_mesh(output_filename, output_mesh_o3d)


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--input_filename', required=True)
  parser.add_argument('--output_dir', required=True)
  parser.add_argument('--marker_diameter_mm', type=float, default=3,
      help="Diameter of marker as ordered")
  parser.add_argument('--recut', action='store_true')
  args = parser.parse_args()
  embed(args.input_filename, args.output_dir, args.marker_diameter_mm, args.recut)
