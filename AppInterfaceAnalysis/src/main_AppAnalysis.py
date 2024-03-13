#!/home/orin1/anaconda3/envs/alpaca/bin/python
# -*- coding:utf-8 _*-
from AppAnalysis.Algorithm0 import LLM_core
import rospy
from std_msgs.msg import String
import codecs
import sys


def main():
    model,tokenizer,device,args=LLM_core.init_model()
    
    rospy.init_node('node1')
    
    current_time = rospy.Time.now()
    
    rospy.loginfo("Current ROS time: %f seconds, AppAnalysis node ", current_time.to_sec())
    
    rospy.spin()
    
if __name__=='__main__':
    main()