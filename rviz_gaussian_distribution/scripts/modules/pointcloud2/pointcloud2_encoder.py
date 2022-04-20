import genpy
import numpy as np
import rospy
import sensor_msgs.msg as sensor_msgs
from ros_numpy.point_cloud2 import array_to_pointcloud2


class PointCloud2Encoder:
    """
    NumpyのarrayをROSのsensor_msgs.PointCloud2に変換するクラス．
    """

    # ==================================================================================================
    #
    #   Public Method
    #
    # ==================================================================================================
    @classmethod
    def xyz_rgba_to_pointcloud2(cls, xyz, rgb, alpha, stamp=None, frame_id="map"):
        """
        XYZ と RGBA からPointCloud2のmsgを生成．

        Args:
            xyz (np.ndarray): [N1, N2, 3] or [N, 3]
            rgb (np.ndarray): [N1, N2, 3] or [N, 3]
            alpha (np.ndarray): [N1, N2, 3] or [N, 3]
            stamp (genpy.Time):
            frame_id (str):

        Returns:
            sensor_msgs.PointCloud2:
        """
        xyz = cls._reshape_pointcloud2_format(xyz)
        rgb = cls._reshape_pointcloud2_format(rgb)

        height, width = xyz.shape[:2]
        cloud_dtype = [("x", np.float32), ("y", np.float32), ("z", np.float32), ("rgba", np.float32)]
        cloud = np.empty((height, width), cloud_dtype)
        cloud["x"] = xyz[:, :, 0]
        cloud["y"] = xyz[:, :, 1]
        cloud["z"] = xyz[:, :, 2]

        rgb = rgb.astype(np.uint32)
        r, g, b = (rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2])
        alpha = alpha.reshape(1, -1).astype(np.uint32)
        rgba_f32 = np.array((r << 0) | (g << 8) | (b << 16) | (alpha << 24), dtype=np.uint32)
        rgba_f32.dtype = np.float32
        cloud["rgba"] = rgba_f32

        if stamp is None:
            stamp = rospy.Time.now()
        return array_to_pointcloud2(cloud, stamp, frame_id)

    @classmethod
    def empty_to_pointcloud2(cls, stamp=None, frame_id="map"):
        """
        XYZ と RGBA からPointCloud2のmsgを生成．

        Args:
            stamp (genpy.Time):
            frame_id (str):

        Returns:
            sensor_msgs.PointCloud2:
        """
        cloud_dtype = [("x", np.float32), ("y", np.float32), ("z", np.float32), ("rgb", np.float32)]
        cloud = np.empty((0, 0), cloud_dtype)

        if stamp is None:
            stamp = rospy.Time.now()

        return array_to_pointcloud2(cloud, stamp, frame_id)

    # ==================================================================================================
    #
    #   Private Method
    #
    # ==================================================================================================
    @staticmethod
    def _reshape_pointcloud2_format(points):
        """
        PointCloud2のフォーマットに合わせるため，3次元でないarrayだった場合に3次元に変換する．

        Args:
            points (np.ndarray): [N1, N2, 3]

        Returns:

        """
        if points.ndim != 3:
            return points.reshape((1, -1, 3))
        return points
