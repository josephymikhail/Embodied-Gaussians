# Copyright (c) 2025 Boston Dynamics AI Institute LLC. All rights reserved.

from dataclasses import dataclass
import typing
import cv2
import numpy as np

import open3d as o3d
from embodied_gaussians.scene_builders.domain import MaskedPosedImageAndDepth


@dataclass
class GroundFinderSettings:
    # parameters include: sampling density, plane dimensions
    # ransac settings (look into these)

    points_per_cm: float = 0.8  # 0.8
    xmin: float = -0.9
    xmax: float = 0.9
    ymin: float = -1.0
    ymax: float = 1.0

    # parameters for ransac plane fitting
    # distance thershold - how far point can be from plane to qualify as inlier
    # ransac_n - number of points sampled in each iteration (3 to define plane)
    # num_iterations - how many times ransac iterates
    # these are all used in the "fit plane" section
    plane_segment_distance_threshold: float = 0.005  # 0.01
    plane_segment_ransac_n: int = 3
    plane_segment_num_iterations: int = 1000  # 1000

    max_depth: float = 10.0


@dataclass
class GroundFinderResult:
    # 4 numbers define plane (stored in ground_plane.json)
    plane: np.ndarray  # (4,) ax + by + cz + d = 0
    points: np.ndarray


class GroundFinder:
    @staticmethod
    def find_ground(
        settings: GroundFinderSettings,
        datapoints: list[MaskedPosedImageAndDepth],
        visualize: bool = False,
    ) -> GroundFinderResult:
        """Find the ground plane from a list of data"""

        # ====================
        # GET ALL POINTCLOUDS
        # ====================
        all_pointclouds = []
        for datapoint in datapoints:
            print("ran through datapoints loop")
            if datapoint.mask is not None:
                datapoint.depth[datapoint.mask == False] = 0.0
            w = datapoint.depth.shape[1]
            h = datapoint.depth.shape[0]
            # loads intrinsics, check these to see if they match intrinsics.py
            # breakpoint()

            # intrinsics = o3d.camera.PinholeCameraIntrinsic(
            #     w,
            #     h,
            #     datapoint.K[0, 0],
            #     datapoint.K[1, 1],
            #     datapoint.K[0, 2],
            #     datapoint.K[1, 2],
            # )
            intrinsics = o3d.camera.PinholeCameraIntrinsic(
                1280, 720, 906.2524, 910.88, 670.619, 397.239
            )
            # print(w)
            # print(h)
            # print(datapoint.K[0, 0])
            # print(type(intrinsics))

            depth_image = o3d.geometry.Image(datapoint.depth)
            if datapoint.image is not None:
                color_image = o3d.geometry.Image(datapoint.image)
                rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                    color_image, depth_image, convert_rgb_to_intensity=False
                )
                pointcloud = o3d.geometry.PointCloud.create_from_rgbd_image(
                    rgbd_image, intrinsics
                )
            else:
                pointcloud = o3d.geometry.PointCloud.create_from_depth_image(
                    depth_image,
                    intrinsics,
                    depth_scale=1.0 / datapoint.depth_scale,
                    depth_trunc=settings.max_depth,
                )
            pointcloud.transform(datapoint.get_X_WC("opencv"))  # originally opencv
            # trying to manually rotate pointcloud to deal with "no points after pruning below ground" error during object generation
            R = pointcloud.get_rotation_matrix_from_xyz((0, np.pi, np.pi))
            pointcloud.rotate(R, center=(0, 0, 0))

            all_pointclouds.append(pointcloud)

        final_pointcloud = o3d.geometry.PointCloud()
        for p in all_pointclouds:
            final_pointcloud += p
        # breakpoint()

        # looking back at my old ground planes, somehow the z axis flipped?
        # i think this is why ive been getting no points when pruning below ground
        # for now i am just manually flipping z coordinates
        # this is not a good fix so later i will look into this more
        # points = np.asarray(final_pointcloud.points)
        # points[:, 2] *= -1
        # final_pointcloud.points = o3d.utility.Vector3dVector(points)

        # ====================
        # FIT PLANE
        # ====================
        # uses ransac to generate plane, then checks pointcloud
        # for all points that lie on that plane (these are inliers)
        # breakpoint()
        plane_model, inliers = final_pointcloud.segment_plane(
            distance_threshold=settings.plane_segment_distance_threshold,
            ransac_n=settings.plane_segment_ransac_n,
            num_iterations=settings.plane_segment_num_iterations,
        )
        inlier_cloud = final_pointcloud.select_by_index(inliers)
        print("check inlier cloud size")

        # ====================
        # GET PLANE POINTCLOUD
        # ====================
        plane_model = np.array(plane_model)
        points_per_cm = settings.points_per_cm
        xmin, xmax, ymin, ymax = (
            settings.xmin,
            settings.xmax,
            settings.ymin,
            settings.ymax,
        )
        num_x = int(abs(xmax - xmin) * points_per_cm * 100)
        num_y = int(abs(ymax - ymin) * points_per_cm * 100)
        x = np.linspace(xmin, xmax, num_x)
        y = np.linspace(ymin, ymax, num_y)
        x, y = np.meshgrid(x, y)
        z = (-plane_model[0] * x - plane_model[1] * y - plane_model[3]) / plane_model[2]
        plane_points = np.stack([x, y, z], axis=-1)
        plane_points = plane_points.reshape(-1, 3)
        plane_points = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(plane_points))

        # ====================
        # CROP PLANE POINTCLOUD
        # ====================
        kdtree = o3d.geometry.KDTreeFlann(inlier_cloud)
        final_points = []
        print(np.asarray(plane_points.points)[0:10])
        for p in plane_points.points:
            [k, idx, _] = kdtree.search_radius_vector_3d(p, 0.01)
            if k >= 1:
                print("k =  " + str(k) + ", p = " + str(p))
                final_points.append(p)
        final_points = np.array(final_points)
        print(type(final_points))
        print(np.array(final_points).shape)
        plane_points = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(final_points))

        res = GroundFinderResult(plane_model, final_points)  # original code

        if visualize:
            origin = o3d.geometry.TriangleMesh.create_coordinate_frame(
                # this line is where the origin is being visualized
                size=1.0,
                origin=[0, 0, 0],
            )
            inlier_cloud.paint_uniform_color([0.0, 1.0, 0.0])
            o3d.visualization.draw_geometries([plane_points, origin, *all_pointclouds])

        return res
