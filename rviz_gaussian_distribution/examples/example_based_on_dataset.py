#! /usr/bin/env python
import random
import actionlib
import numpy as np
import rospy
import rviz_gaussian_distribution_msgs.msg as rgd_msgs
import rviz_display_words_distribution.msg as rviz_display_words_distribution_msg
import std_msgs.msg as std_msgs
from modules import dataset


# ==================================================================================================
#
#   ROS Publisher Method
#
# ==================================================================================================
def publish_distribution(i, mu_set, sigma_set, r, g, b):

    # Calc parameters of Gaussian distribution．
    mu_x = mu_set[i][0]
    mu_y = mu_set[i][1]
    sigma_x = np.sqrt(sigma_set[i][0][0])
    sigma_y = np.sqrt(sigma_set[i][1][1])
    covariance = sigma_set[i][0][1]

    # Publish
    publisher = rospy.Publisher("/input/add", rgd_msgs.GaussianDistribution, queue_size=1)
    rospy.sleep(0.5)

    msg = rgd_msgs.GaussianDistribution(
        mean_x=mu_x-5.0, mean_y=mu_y,
        std_x=sigma_x, std_y=sigma_y,
        covariance=covariance,
        mixture_ratio=1.0,
        r=r, g=g, b=b,
        id=i
    )
    publisher.publish(msg)


def publish_remove(id_):
    """
    Remove published distributions
    Args:
        id_ (int):
    """
    publisher = rospy.Publisher("/input/remove", std_msgs.Int16, queue_size=1)
    rospy.sleep(0.1)

    # Publish
    msg = std_msgs.Int16(id_)
    publisher.publish(msg)


def publish_clear():
    """
    Clear published distributions
    """
    publisher = rospy.Publisher("/input/clear", std_msgs.Empty, queue_size=1)
    test_publisher = rospy.Publisher("/place_object_word/input/clear", std_msgs.Empty, queue_size=1)
    rospy.sleep(0.5)

    msg = std_msgs.Empty()
    publisher.publish(msg)
    test_publisher.publish(msg)



def pub_words_objects_dist(_id, mu, words, words_dist, objects, objects_dist, r, g, b):
    pub = rospy.Publisher('/place_object_word/input/add', rviz_display_words_distribution_msg.WordObjectDistribution, queue_size=1)    
    rospy.sleep(0.5)
    
    msg = rviz_display_words_distribution_msg.WordObjectDistribution(    
        id=_id,
        mean_x = mu[i][0]-5.0,
        mean_y = mu[i][1],    
        words=words[0],
        words_dist=words_dist[i],
        objects=objects[0],
        objects_dist=objects_dist[i],
        r=r, g=g, b=b
    )
    
    pub.publish(msg)

# ==================================================================================================
#
#   ROS Action Method
#
# ==================================================================================================
def action_distribution(i, mu_set, sigma_set):
    """
    Publish Gaussian distributions using Actionlib
    Args:
        id_ (int):
    """

    mu_x = mu_set[i][0]
    mu_y = mu_set[i][1]
    sigma_x = np.sqrt(sigma_set[i][0][0])
    sigma_y = np.sqrt(sigma_set[i][1][1])
    covariance = sigma_set[i][0][1]
    r = random.randint(0, 255)
    g = random.randint(0, 255)
    b = random.randint(0, 255)

    # Publish
    add_action = actionlib.SimpleActionClient("/action/add", rgd_msgs.AddAction)
    add_action.wait_for_server()
    msg = rgd_msgs.GaussianDistribution(
        mean_x=mu_x, mean_y=mu_y,
        std_x=sigma_x, std_y=sigma_y,
        covariance=covariance,
        r=r, g=g, b=b,
        id=i
    )
    add_action.send_goal(rgd_msgs.AddGoal(msg))
    add_action.wait_for_result()


def action_remove(id_):
    """
    Remove published distributions using Actionlib
    Args:
        id_ (int):
    """
    remove_action = actionlib.SimpleActionClient("/action/remove", rgd_msgs.RemoveAction)
    remove_action.wait_for_server()

    remove_action.send_goal(rgd_msgs.RemoveGoal(id_))
    remove_action.wait_for_result()


def action_clear():
    """
    Clear published distributions using Actionlib
    """
    clear_action = actionlib.SimpleActionClient("/action/clear", rgd_msgs.ClearAction)
    clear_action.wait_for_server()

    clear_action.send_goal(rgd_msgs.ClearGoal())
    clear_action.wait_for_result()


# ==================================================================================================
#
#   __main__
#
# ==================================================================================================
if __name__ == "__main__":
    rospy.init_node("example_client")
    rospy.sleep(2.0)

    colors = [(255, 0, 0), (167, 87, 168), (255, 165, 0), (234, 145, 152), (254, 220, 189), (255, 244, 80)]

    mu_set, K = dataset.read_mean_of_Gaussian_distribution()
    sigma_set = dataset.read_variance_of_Gaussian_distribution()
    phi_set = dataset.read_mixture_ratio_of_Gaussuan_distribution()
    words = dataset.read_words_data()    
    words_dist = dataset.read_words_distribution()    
    object_words = dataset.read_object_words_data()    
    object_words_dist = dataset.read_object_words_distribution()

    # print("mu:{}".format(mu_set))
    # print("sigma_set:{}".format(sigma_set))
    print("K:{}".format(K))

    # Publish Gaussian distributions
    for i in range(K):
        if not rospy.is_shutdown():
            b, g, r = colors.pop()
            publish_distribution(i, mu_set, sigma_set, r, g, b)
            pub_words_objects_dist(i, mu_set, [[]], words_dist, object_words, object_words_dist, r, g, b)
            rospy.sleep(1.0)
        
    rospy.sleep(10.0)

    publish_clear()

    colors = [(255, 0, 0), (167, 87, 168), (255, 165, 0), (234, 145, 152), (254, 220, 189), (255, 244, 80)]

    # Publish Gaussian distributions
    for i in range(K):
        if not rospy.is_shutdown():
            b, g, r = colors.pop()
            publish_distribution(i, mu_set, sigma_set, r, g, b)
            pub_words_objects_dist(i, mu_set, words, words_dist, [[]], object_words_dist, r, g, b)
            rospy.sleep(1.0)
        
    rospy.sleep(10.0)

    publish_clear()

    colors = [(255, 0, 0), (167, 87, 168), (255, 165, 0), (234, 145, 152), (254, 220, 189), (255, 244, 80)]

    # Publish Gaussian distributions
    for i in range(K):
        if not rospy.is_shutdown():
            b, g, r = colors.pop()
            publish_distribution(i, mu_set, sigma_set, r, g, b)
            pub_words_objects_dist(i, mu_set, words, words_dist, object_words, object_words_dist, r, g, b)
            rospy.sleep(1.0)
        
    rospy.sleep(10.0)

    publish_clear()
