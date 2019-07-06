import open3d as o3d
import copy
import numpy as np
import transforms3d.euler as txe


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, with_scaling=False):
    print(":: RANSAC registration on downsampled point clouds.")
    distance_threshold = 3 ## mm
    result = o3d.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        o3d.registration.TransformationEstimationPointToPoint(with_scaling), 3,
        [
            o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ],
        o3d.registration.RANSACConvergenceCriteria(4000000, 500000))
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
            feats[idx, nns+nv_idx*3 : nns+(nv_idx+1)*3] =\
                pt - pts[nn_idxs[nv_idx]]

    out = o3d.registration.Feature()
    out.resize(feat_dim, len(pts))
    out.data = feats.T
    return out


def register(source, target):
    source_fpfh = compute_feature(source)
    target_fpfh = compute_feature(target)
    result = execute_global_registration(source, target, source_fpfh,
                                         target_fpfh)
    print('# inliers = {:d}, Inlier RMSE = {:f} mm'.
          format(len(result.correspondence_set), result.inlier_rmse))
    print('Recovered transform = ')
    print(result.transformation)
    draw_registration_result(source, target, result.transformation)
    return result.transformation.copy()


if __name__ == '__main__':
    size = 10
    source = np.random.uniform(size=(8, 3)) * size
    permute_idx = np.random.permutation(len(source))
    target = source.copy()[permute_idx]

    R = txe.euler2mat(*(2*np.pi*np.random.uniform(size=3)))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = np.random.uniform(size=3)*size/2

    src = o3d.geometry.PointCloud()
    src.points = o3d.utility.Vector3dVector(source)
    dst = o3d.geometry.PointCloud()
    dst.points = o3d.utility.Vector3dVector(target)

    dst.transform(T)

    register(src, dst)
    print('Original transform = ')
    print(T)