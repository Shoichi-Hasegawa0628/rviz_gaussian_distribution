#! /usr/bin/env python
import random
import actionlib
import numpy as np
import rospy
import roslib.packages
import rviz_gaussian_distribution_msgs.msg as rgd_msgs
import std_msgs.msg as std_msgs


path = roslib.packages.get_pkg_dir('rviz_gaussian_distribution') + '/data'


def read_mean_of_Gaussian_distribution():
    """
    @return:
    """
    all_mu = []
    K = 0
    for line in open(
            path+"/mu.csv",'r'):
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
            path+"/sig.csv",'r'):
        data = line[:].split(',')
        # sigma=[] #(x,y,sin,cos)
        sigma = [[float(data[0]), float(data[1]), 0, 0], [float(data[2]), float(data[3]), 0, 0], [0, 0, 0, 0],
                 [0, 0, 0, 0]]
        all_sigma.append(sigma)
    return all_sigma

def read_mixture_ratio_of_Gaussuan_distribution():
    """
    @return:
    """
    all_phi = []

    for line in open(path+'/phi.csv', 'r'):
        data = line.split(',')

        phi = [float(data[0]), float(data[1]), float(data[2])]
        all_phi.append(phi)

    return all_phi

def read_words_data():    
    all_words = []    
    
    for line in open(path+'/W_list.csv'):    
        data = line.split(',')    
        data.pop()    
    
        all_words.append(data)    
    
    return all_words    
    
def read_words_distribution():    
    all_words_dist = []    
    
    for line in open(path+'/W.csv'):    
        data = line.replace(',\n', '').split(',')    
        data = list(map(float, data))    
    
        all_words_dist.append(data)    
    
    return all_words_dist    
    
def read_object_words_data():    
    all_object_words = []    
    
    for line in open(path+'/Object_W_list.csv'):    
        data = line.replace('\n', '').split(',')    
    
        all_object_words.append(data)    
    
    return all_object_words    
    
def read_object_words_distribution():    
    all_object_words_dist = []    
    
    for line in open(path+'/Xi.csv'):    
        data = line.replace(',\n', '').split(',')    
        data = list(map(float, data))    

        all_object_words_dist.append(data)

    return all_object_words_dist
