#!/home/orin1/anaconda3/envs/alpaca/bin/python
# -*- coding:utf-8 _*-
from AppAnalysis.Algorithm0.LLM_instance import LLM
import rospy
from std_msgs.msg import String
import codecs
import sys


def main():

    rospy.init_node('AppAnalysis_node')

    current_time = rospy.Time.now()

    rospy.loginfo("LLM_instance初始化...............")

    LLM_instance=LLM()
    
    LLM_instance.Init()
    
    LLM_instance.Start()
    
    LLM_instance.End()

    rospy.spin()


if __name__ == '__main__':
    main()
