from AppAnalysis.Algorithm0 import LLM_core
import rospy
from std_msgs import String
import codecs
import sys

sys.stdout = codecs.getwriter("utf-8")(sys.stdout.detach())
sys.stderr = codecs.getwriter("utf-8")(sys.stderr.detach())


def main():
    model,tokenizer,device,args=LLM_core.init_model()
    
    rospy.init_node('AppAnalysis_node')
    
    rospy.Time.init()
    
    current_time = rospy.Time.now()
    
    rospy.loginfo("Current ROS time: %f seconds, AppAnalysis node ", current_time.to_sec())
    
    rospy.spin()