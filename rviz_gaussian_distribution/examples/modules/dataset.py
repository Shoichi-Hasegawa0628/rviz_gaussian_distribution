#! /usr/bin/env python
import random
import actionlib
import numpy as np
import rospy
import rviz_gaussian_distribution_msgs.msg as rgd_msgs
import std_msgs.msg as std_msgs


def read_mean_of_Gaussian_distribution():
    """

    @return:
    """
    all_mu = []
    K = 0
    for line in open(
            "/root/HSR/mu.csv",'r'):
        mu = []  # (x,y,sin,cos)
        data = line[:].split(',')
        mu += [float(data[0])]
        mu += [float(data[1])]
        mu += [0]  # float(data[2])]
        mu += [0]  # float(data[3])]
        all_mu.append(mu)
        K += 1
    return all_mu, K


def read_variance_of_Gaussian_distribution():
    all_sigma = []
    for line in open(
            "/root/HSR/sig.csv",'r'):
        data = line[:].split(',')
        # sigma=[] #(x,y,sin,cos)
        sigma = [[float(data[0]), float(data[1]), 0, 0], [float(data[2]), float(data[3]), 0, 0], [0, 0, 0, 0],
                 [0, 0, 0, 0]]
        all_sigma.append(sigma)
    return all_sigma


if __name__ == "__main__":
    pass
