#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import numpy as np
import pandas as pd
import tensorflow as tf
import tensorflow_addons as tfa
from keras.models import Model, Sequential, load_model
from keras.layers import Input, Activation, Dense
from keras.optimizers import SGD
from keras.utils.np_utils import to_categorical
from sklearn.model_selection import train_test_split
from keras import backend as K

from object_recognition_pkg.msg import data_completed

pub = rospy.Publisher('/data_predicted',  data_completed)


def callback(params):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s ", params.params)
    # rospy.loginfo(rospy.get_caller_id() + "I heard %d ", params.id[0])

    # data = pd.read_csv("new-v2-after-completing-comb.csv", sep=',', decimal=',')
    # data = pd.read_csv("drive/MyDrive/dataset/new-from-windows.csv", sep=';')
    data = pd.read_csv("newDatasetv6.5.csv", sep=';')
    x = data.drop(data.columns[[0]], axis=1, inplace=False)
    y = pd.get_dummies(data["label"])

    # x_train, x_test, y_train, y_test = train_test_split(x, y, test_size=0.33)


    new_model = load_model(
        'model-own-dataset-ourcvfh.h5',  
        custom_objects= {
            'F1Score': tfa.metrics.F1Score(num_classes=5, average="micro")
        }
    )

    # print(params.vfhs.shape)
    # rospy.loginfo(len(params.vfhs))

    # predicted = new_model.predict(x_test) 
    i = 0
    restructured_vfhs = [ [ 0 ] * 308 ] * len(params.vfhs) #create fixed list (array in python) to restructre 
    while i < len(params.vfhs):
        j = 0
        while j < len(params.vfhs[i].vfh):
            restructured_vfhs[i][j] = params.vfhs[i].vfh[j]
            j += 1
        i += 1
    
    # make it to np array 
    restructured_vfhs = np.array(restructured_vfhs)
    # print(restructured_vfhs.shape)

    predicted = new_model.predict(restructured_vfhs)

    msg = data_completed()

    for i in range (0, predicted.shape[0]):
        index_max = np.argmax(predicted[i])
        label_predicted = y.columns[index_max]
        score_predicted = (predicted[i][index_max]*100)
        print(str(i) + ". " + label_predicted + " " + "{:.2f}".format(score_predicted) + " %")
        # msg.label[i] = label_predicted
        # msg.score[i] = score_predicted
        msg.label.append(label_predicted)
        msg.score.append(score_predicted)

    # for i in data.vfh:
        # rospy.loginfo(data.vfh(i))

    rate = rospy.Rate(10) #10 Hz


# while not rospy.is_shutdown():
    msg.id = params.id        
    # msg.clustered = params.clustered
    msg.vfhs = params.vfhs
    # msg.rawRGBD = params.rawRGBD
    msg.bboxs = params.bboxs
    msg.test = "test"
    # msg.label
    # msg.score
    rospy.loginfo("data published")
    pub.publish(msg)
    # rate.sleep()
    


def listener():
    #in ROS, Nodes are uniquelly named. if two nodes with the same
    #name are launched, the previous one is kikcked off. the
    #anonymous=True flag means that rospy will choose a unique
    #name for our 'listener' node so that multiple listeners can
    #run simultanously
    rospy.init_node('subcriber_node', anonymous=True)

    # rospy.Subscriber("chatter", String, callback)
    rospy.Subscriber("/data_completed", data_completed, callback)

    #spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def talker():
    # pub = rospy.Publisher('chatter',  String, queue_size=10)
    pub = rospy.Publisher('/data_predicted',  data_completed)
    rospy.init_node('subcriber_node', anonymous=True)
    rate = rospy.Rate(10) #10 Hz

    # str = ['laskdjalksjd','asd']

    # msg = data_completed()
    # msg.label[0] = "hello"
    # msg.label[1] = "hello1"
    # msg.label.append("hello")
    # msg.label.append("hello1")
    
    # msg.test_array_string = str
    # pub.publish(msg)
    

if __name__ == '__main__':
    print("hellow rold")
    try:
        talker()
    except rospy.ROSInternalException:
        pass
    listener()