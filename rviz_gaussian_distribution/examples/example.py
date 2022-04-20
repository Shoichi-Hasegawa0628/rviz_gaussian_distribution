#! /usr/bin/env python
import random

import actionlib
import numpy as np
import rospy
import rviz_gaussian_distribution_msgs.msg as rgd_msgs
import std_msgs.msg as std_msgs


class ExampleNode:

    def __init__(self):
        self._is_animation = rospy.get_param("~is_animation", default=True)

        # Publisher
        self._add_publisher = rospy.Publisher("/input/add", rgd_msgs.GaussianDistribution, queue_size=1)
        self._remove_publisher = rospy.Publisher("/input/remove", std_msgs.Int16, queue_size=1)
        self._clear_publisher = rospy.Publisher("/input/clear", std_msgs.Empty, queue_size=1)

        # ActionServer
        self._add_action = actionlib.SimpleActionClient("/action/add", rgd_msgs.AddAction)
        self._remove_action = actionlib.SimpleActionClient("/action/remove", rgd_msgs.RemoveAction)
        self._clear_action = actionlib.SimpleActionClient("/action/clear", rgd_msgs.ClearAction)
        self._add_action.wait_for_server()
        self._remove_action.wait_for_server()
        self._clear_action.wait_for_server()

    # ==================================================================================================
    #
    #   ROS Publisher Method
    #
    # ==================================================================================================
    def publish_distribution(self, id_):
        """
        メイン処理．
        アニメーションをしない場合はSpin

        Args:
            id_ (int):

        """

        # ランダムな点群を生成．
        center_x = random.random() * 4 - 2
        center_y = random.random() * 4 - 2
        x = center_x + (np.random.random(size=100) * (0.5 + random.random() * 2))
        y = center_y + (np.random.random(size=100) * (0.5 + random.random() * 2))

        # 生成された点群の分布パラメータを計算．
        mu_x = np.mean(x)
        mu_y = np.mean(y)
        sigma_x = np.std(x)
        sigma_y = np.std(y)
        covariance = (np.sum(x * y) / len(x)) - (mu_x * mu_y)
        r = random.randint(0, 255)
        g = random.randint(0, 255)
        b = random.randint(0, 255)

        msg = rgd_msgs.GaussianDistribution(
            mean_x=mu_x, mean_y=mu_y, std_x=sigma_x, std_y=sigma_y, covariance=covariance, r=r, g=g, b=b, id=id_
        )
        self._add_publisher.publish(msg)

    def publish_remove(self, id_):
        self._remove_publisher.publish(std_msgs.Int16(id_))

    def publish_clear(self):
        self._clear_publisher.publish(std_msgs.Empty())

    # ==================================================================================================
    #
    #   ROS Action Method
    #
    # ==================================================================================================
    def action_distribution(self, id_):
        """
        メイン処理．
        アニメーションをしない場合はSpin

        Args:
            id_ (int):

        """

        # ランダムな点群を生成．
        center_x = random.random() * 5
        center_y = random.random() * 5
        x = center_x + (np.random.random(size=100) * (0.5 + random.random() * 2))
        y = center_y + (np.random.random(size=100) * (0.5 + random.random() * 2))

        # 生成された点群の分布パラメータを計算．
        mu_x = np.mean(x)
        mu_y = np.mean(y)
        sigma_x = np.std(x)
        sigma_y = np.std(y)
        covariance = (np.sum(x * y) / len(x)) - (mu_x * mu_y)
        r = random.randint(0, 255)
        g = random.randint(0, 255)
        b = random.randint(0, 255)

        msg = rgd_msgs.GaussianDistribution(
            mean_x=mu_x, mean_y=mu_y, std_x=sigma_x, std_y=sigma_y, covariance=covariance, r=r, g=g, b=b, id=id_
        )
        self._add_action.send_goal(rgd_msgs.AddGoal(msg))
        self._add_action.wait_for_result()

    def action_remove(self, id_):
        self._remove_action.send_goal(rgd_msgs.RemoveGoal(id_))
        self._remove_action.wait_for_result()

    def action_clear(self):
        self._clear_action.send_goal(rgd_msgs.ClearGoal())
        self._clear_action.wait_for_result()


# ==================================================================================================
#
#   __main__
#
# ==================================================================================================
if __name__ == "__main__":
    rospy.init_node("example_client")
    node = ExampleNode()

    rospy.sleep(1.0)

    # Publish Gaussian distributions
    for i in range(10):
        if not rospy.is_shutdown():
            node.publish_distribution(i)
            rospy.sleep(1)

    # Remove published distributions
    for i in range(5):
        if not rospy.is_shutdown():
            node.publish_remove(i)
            rospy.sleep(1.5)

    # Clear published distributions
    node.publish_clear()

    # Action send Gaussian distributions
    for i in range(10):
        if not rospy.is_shutdown():
            node.action_distribution(i)
            rospy.sleep(1.5)

    # Remove published distributions
    for i in range(5):
        if not rospy.is_shutdown():
            node.action_remove(i)
            rospy.sleep(1.5)

    # Clear published distributions
    node.action_clear()
