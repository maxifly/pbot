import rospy
from std_msgs.msg import String

pub = rospy.Publisher('beeper_topic', String, queue_size=10)
rospy.init_node('beeper_topic_publisher')
rospy.loginfo("Hello from PUB node")
r = rospy.Rate(10) # 10hz

while not rospy.is_shutdown():
    pub.publish("Hello World")
    r.sleep()
