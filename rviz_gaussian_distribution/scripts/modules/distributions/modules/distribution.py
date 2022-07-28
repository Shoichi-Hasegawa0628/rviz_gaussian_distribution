import numpy as np
import rospy

from ...pointcloud2 import PointCloud2Encoder
import sensor_msgs.msg as sensor_msgs


class Distribution:
    """
    可視化に必要な分布の各種情報を持つクラス．
    """

    def __init__(self, xyz, rgb, alpha):
        """

        Args:
            xyz (np.ndarray): [N, 3]
            rgb (np.ndarray): [N, 3]
            alpha (np.ndarray): [N, 1]

        """
        self._xyz = xyz
        self._rgb = rgb
        self._alpha = alpha

    @property
    def xyz(self):
        return self._xyz

    @property
    def rgb(self):
        return self._rgb

    @property
    def alpha(self):
        return self._alpha

    def to_point_cloud_2d_msg(self, frame_id="map"):
        """
        XYZ, RGB, Alpha から PointCloud2のmsgを生成．

        Args:
            frame_id (str):

        Returns:
            sensor_msgs.PointCloud2:

        """
        return PointCloud2Encoder.xyz_rgba_to_pointcloud2(self._xyz, self._rgb, self._alpha, rospy.Time.now(), frame_id)
