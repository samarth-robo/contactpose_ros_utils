import open3d as o3d
import copy
import numpy as np
import transforms3d.euler as txe
from itertools import combinations
from scipy.special import comb


def draw_registration_result(source, target, source_mesh, transformation):
  source_temp = copy.deepcopy(source)
  target_temp = copy.deepcopy(target)
  source_temp.paint_uniform_color([1, 0.706, 0])
  target_temp.paint_uniform_color([0, 0.651, 0.929])
  target_temp.transform(np.linalg.inv(transformation))
  source_mesh.compute_vertex_normals()
  o3d.visualization.draw_geometries([source_mesh, source_temp, target_temp])


def execute_global_registration(source, target, source_fpfh, target_fpfh,
    distance_thresh, with_scaling=False):
  assert len(source.points) == len(target.points)
  result = o3d.registration.registration_ransac_based_on_feature_matching(
    source, target, source_fpfh, target_fpfh, distance_thresh,
    o3d.registration.TransformationEstimationPointToPoint(with_scaling), 3,
    [
      o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
      o3d.registration.CorrespondenceCheckerBasedOnDistance(
        distance_thresh)
    ])
  return result


def compute_feature(pc, nns=-1, nvs=2):
  if nns < 0:
    nns = len(pc.points)-1
  else:
    nns = min(nns, len(pc.points)-1)

  feat_dim = nns + 3*nvs
  o3dg = o3d.geometry
  kdtree = o3dg.KDTreeFlann(pc)
  pts = np.asarray(pc.points)
  feats = np.zeros((len(pts), feat_dim))
  for idx, pt in enumerate(pts):
    # already sorted by distance
    _, nn_idxs, nn_dists = kdtree.search_knn_vector_3d(pt, nns+1)
    nn_idxs, nn_dists = nn_idxs[1:], nn_dists[1:]
    feats[idx, :nns] = nn_dists
    for nv_idx in range(nvs):
      feats[idx, nns+nv_idx*3 : nns+(nv_idx+1)*3] = \
        pt - pts[nn_idxs[nv_idx]]

  out = o3d.registration.Feature()
  out.resize(feat_dim, len(pts))
  out.data = feats.T
  return out


def register(source, target, source_mesh, n_source_markers,
    distance_thresh_mm=3.0):
  assert n_source_markers <= len(target.points)
  assert len(source.points) >= n_source_markers
  n_source_iters = comb(len(source.points), n_source_markers, exact=True)
  n_target_iters = comb(len(target.points), n_source_markers, exact=True)
  source_points = np.asarray(source.points)
  target_points = np.asarray(target.points)
  result = o3d.registration.RegistrationResult()
  s_iter = 1
  for source_idx in combinations(range(len(source.points)), n_source_markers):
    print(''.format(s_iter, n_source_iters))
    source_idx = list(source_idx)
    this_source = o3d.geometry.PointCloud()
    this_source.points = o3d.utility.Vector3dVector(source_points[source_idx])
    source_fpfh = compute_feature(this_source)
    t_iter = 1
    for target_idx in combinations(range(len(target.points)), n_source_markers):
      print('Target Iter {:d} / {:d}, Source Iter {:d} / {:d}'.format(t_iter,
        n_target_iters, s_iter, n_source_iters))
      target_idx = list(target_idx)
      this_target = o3d.geometry.PointCloud()
      this_target.points = o3d.utility.Vector3dVector(target_points[target_idx])
      target_fpfh = compute_feature(this_target)
      this_result = execute_global_registration(this_source, this_target,
        source_fpfh, target_fpfh, distance_thresh_mm)
      if (this_result.fitness > result.fitness) or \
        (this_result.fitness == result.fitness and
         this_result.inlier_rmse < result.inlier_rmse):
        print('Better solution found')
        result = this_result
      t_iter += 1
    s_iter += 1
  print('# inliers = {:d}, Inlier RMSE = {:f} mm'.
        format(len(result.correspondence_set), result.inlier_rmse))
  print('Recovered transform = ')
  print(result.transformation)
  draw_registration_result(source, target, source_mesh, result.transformation)
  return result.transformation.copy()


if __name__ == '__main__':
  size = 10
  source = np.random.uniform(size=(8, 3)) * size
  permute_idx = np.random.permutation(len(source))
  target = source.copy()[permute_idx]

  target = np.vstack((target, np.random.uniform(size=(3,3))*size))
  permute_idx = np.random.permutation(len(target))
  target = target[permute_idx]

  R = txe.euler2mat(*(2*np.pi*np.random.uniform(size=3)))
  T = np.eye(4)
  T[:3, :3] = R
  T[:3,  3] = np.random.uniform(size=3)*size/2

  src = o3d.geometry.PointCloud()
  src.points = o3d.utility.Vector3dVector(source)
  dst = o3d.geometry.PointCloud()
  dst.points = o3d.utility.Vector3dVector(target)

  dst.transform(T)

  register(src, dst, len(source))
  print('Original transform = ')
  print(T)