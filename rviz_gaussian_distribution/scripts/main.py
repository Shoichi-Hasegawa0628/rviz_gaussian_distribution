#! /usr/bin/env python

import actionlib
import numpy as np
import rospy
import rviz_gaussian_distribution_msgs.msg as rgd_msgs
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs

from modules.distributions import AnimatedDistributionManager, DistributionManager
from modules.distributions.modules import Distribution
from modules.pointcloud2 import PointCloud2Encoder


class VisualizerNode:

    def __init__(self):
        self._is_animation = rospy.get_param("~is_animation", default=True)
        self._animation_time = rospy.get_param("~animation_time", default=1)

        # Publisher
        self._publisher = rospy.Publisher("/output", sensor_msgs.PointCloud2, queue_size=1)

        # Subscriber
        rospy.Subscriber("/input/add", rgd_msgs.GaussianDistribution, self._distribution_callback)
        rospy.Subscriber("/input/remove", std_msgs.Int16, self._unregister_callback)
        rospy.Subscriber("/input/clear", std_msgs.Empty, self._clear_callback)

        # ActionServer
        self._add_action_server = actionlib.SimpleActionServer(
            "/action/add", rgd_msgs.AddAction, self._add_action_callback, auto_start=False
        )
        self._remove_action_server = actionlib.SimpleActionServer(
            "/action/remove", rgd_msgs.RemoveAction, self._remove_action_callback, auto_start=False
        )
        self._clear_action_server = actionlib.SimpleActionServer(
            "/action/clear", rgd_msgs.ClearAction, self._clear_action_callback, auto_start=False
        )

        # Manager
        if self._is_animation:
            self._manager = AnimatedDistributionManager(animated_time=self._animation_time)
        else:
            self._manager = DistributionManager()

        # Start ActionServer
        self._add_action_server.start()
        self._remove_action_server.start()
        self._clear_action_server.start()

    # ==================================================================================================
    #
    #   Main Method
    #
    # ==================================================================================================
    def main(self):
        """
        メイン処理．
        アニメーションをしない場合はSpin

        """
        if isinstance(self._manager, AnimatedDistributionManager):
            while not rospy.is_shutdown():
                if not self._manager.is_complete():
                    self._manager.update()
                    self.publish()

                rospy.sleep(0.1)
        else:
            rospy.spin()

    # ==================================================================================================
    #
    #   ROS Publish
    #
    # ==================================================================================================
    def publish(self):
        """
        全ての分布をPublish

        """
        if self._manager.is_empty():
            cloud_msg = PointCloud2Encoder.empty_to_pointcloud2(frame_id="map")
        else:
            distributions = self._manager.get_distributions()
            integrated = self.integrate_visualized_distributions(distributions)
            cloud_msg = integrated.to_point_cloud_2d_msg(frame_id="map")

        self._publisher.publish(cloud_msg)

    # ==================================================================================================
    #
    #   ROS Subscriber Callback
    #
    # ==================================================================================================
    def _distribution_callback(self, msg):
        """
        ガウス分布の登録用のコールバック関数．

        Args:
            msg (rgd_msgs.GaussianDistribution):

        """
        rospy.loginfo(f"<Subscribe>: Add {msg.id}")
        xyz = self._make_visualized_xyz(
            mean_x=msg.mean_x,
            mean_y=msg.mean_y,
            std_x=msg.std_x,
            std_y=msg.std_y,
            covariance=msg.covariance,
        )
        rgb = np.full_like(xyz, fill_value=(msg.r, msg.g, msg.b))
        alpha = self.calc_alpha(xyz[..., 2])
        visualized_distribution = Distribution(xyz, rgb, alpha)

        self._manager.register(msg.id, visualized_distribution)
        self.publish()

    def _unregister_callback(self, msg):
        """
        指定のIDが割り振られたガウス分布の削除用のコールバック関数．

        Args:
            msg (std_msgs.Int16):

        """
        rospy.loginfo(f"<Subscribe>: Remove {msg.data}")
        self._manager.unregister(msg.data)
        self.publish()

    def _clear_callback(self, _):
        """
        ガウス分布の全消去用のコールバック関数．

        Args:
            _ (std_msgs.Empty):

        """
        rospy.loginfo(f"<Subscribe>: Clear")
        self._manager.clear()
        self.publish()

    # ==================================================================================================
    #
    #   ROS Action Callback
    #
    # ==================================================================================================
    def _add_action_callback(self, goal):
        """
        ガウス分布の登録用のアクション版コールバック関数．

        Args:
            goal (rgd_msgs.AddGoal):

        """
        self._distribution_callback(goal.distribution)

        self._add_action_server.set_succeeded(rgd_msgs.AddResult())
        rospy.loginfo(f"<Action>: Add {goal.distribution.id}")

    def _remove_action_callback(self, goal):
        """
        指定のIDが割り振られたガウス分布の削除用のアクション版コールバック関数．

        Args:
            goal (rgd_msgs.RemoveGoal):

        """
        self._manager.unregister(goal.id)
        self.publish()

        self._remove_action_server.set_succeeded(rgd_msgs.RemoveResult())
        rospy.loginfo(f"<Action>: Remove {goal.id}")

    def _clear_action_callback(self, _):
        """
        ガウス分布の全消去用のアクション版コールバック関数．

        Args:
            _ (rgd_msgs.ClearGoal):

        """
        self._manager.clear()
        self.publish()

        self._clear_action_server.set_succeeded(rgd_msgs.ClearResult())
        rospy.loginfo(f"<Action>: Clear")

    # ==================================================================================================
    #
    #   Static Method
    #
    # ==================================================================================================
    @classmethod
    def _make_visualized_xyz(cls, mean_x, mean_y, std_x, std_y, covariance):
        """
        ガウス分布のパラメータから確率密度関数に従って可視化座標群を計算．

        Args:
            mean_x (float):
            mean_y (float):
            std_x (float):
            std_y (float):
            covariance (float):

        Returns:
            np.ndarray: [N, 3]

        """
        rho = covariance / (std_x * std_y)

        x, y = cls.calc_visualization_range(mean_x, mean_y, std_x, std_y)

        h = 1 / (2 * np.pi * std_x * std_y * np.sqrt(1 - rho ** 2))
        g = -1 / (2 * (1 - rho ** 2))

        ex = ((x - mean_x) ** 2) / (std_x ** 2)
        ey = ((y - mean_y) ** 2) / (std_y ** 2)
        exy = (2 * rho * (x - mean_x) * (y - mean_y)) / (std_x * std_y)

        z = h * np.exp(g * (ex + ey - exy))
        xyz = np.stack([x, y, z], axis=-1).reshape(-1, 3)

        return xyz

    @staticmethod
    def integrate_visualized_distributions(distributions):
        """
        1つのPointCloud2のmsgとしてPublishするために複数の分布を1つにまとめる．

        Args:
            distributions (list[Distribution]):

        Returns:
            Distribution:

        """
        xyz_list = []
        rgb_list = []
        alpha_list = []
        for distribution in distributions:
            xyz_list.append(distribution.xyz.reshape(-1, 3))
            rgb_list.append(distribution.rgb.reshape(-1, 3))
            alpha_list.append(distribution.alpha.reshape(-1, 1))

        return Distribution(
            np.concatenate(xyz_list, axis=0), np.concatenate(rgb_list, axis=0), np.concatenate(alpha_list, axis=0)
        )

    @staticmethod
    def calc_visualization_range(mu_x, mu_y, std_x, std_y, n=100):
        """
        分布の描画範囲の決定．
        分布中心から -2σ ~ 2σ (σ=標準偏差)の範囲に分布の95.4%が含まれる．

        Args:
            mu_x (float):
            mu_y (float):
            std_x (float):
            std_y (float):
            n (int):

        Returns:
            (np.ndarray[np.float], np.ndarray[np.float]): [N, N], [N, N]

        """
        min_x = (mu_x - (3 * std_x))
        max_x = (mu_x + (3 * std_x))

        min_y = (mu_y - (3 * std_y))
        max_y = (mu_y + (3 * std_y))

        x = np.linspace(min_x, max_x, num=n)
        y = np.linspace(min_y, max_y, num=n)
        x, y = np.meshgrid(x, y)
        return x, y

    @staticmethod
    def calc_alpha(z):
        """
        透明度(Alpha値)の計算．
        補間曲線の関数を使って透明領域を減らしている．

        Args:
            z (np.ndarray[np.float]): [N]

        Returns:
            np.ndarray[np.uint8]: [N]

        """
        z = z / np.max(z)
        alpha = (z * (2 - z)) * 255.0

        return alpha.astype(np.uint8)


# ==================================================================================================
#
#   __main__
#
# ==================================================================================================
if __name__ == "__main__":
    rospy.init_node("visualization_of_probability_distribution")
    node = VisualizerNode()
    node.main()
