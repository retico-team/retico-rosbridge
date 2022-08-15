import rclpy
import sys
sys.path.append("..")
from retico_ros2.ros2_subscriber import ROS2SubscriberModule
from retico_textoutput.textoutput import TextOutputModule
from retico_ros2.joint_angles_to_text import JointAnglesToTextModule
from trajectory_msgs.msg import JointTrajectory


def traj_to_iu_payload(traj:JointTrajectory):
    return traj.points[0].positions

print("Instantiating Modules...")

rclpy.init()
m1 = ROS2SubscriberModule(ros_node_name='subscriber_joint_trajectory_controller',
                        topic_name="/joint_trajectory_controller/joint_trajectory", 
                        msg_type=JointTrajectory,
                        msg_to_iu_payload=traj_to_iu_payload,
                        debug=True
                        )

m2 = JointAnglesToTextModule()
m3 = TextOutputModule() 

def exit_callback(TextToJointAnglesModule, event_name="exit", data=None):
    print("Exiting...")
    m1.stop()
    m2.stop()
    m3.stop()

print("Creating the Network...")
m1.subscribe(m2)
m2.subscribe(m3)

m1.event_subscribe(event_name='exit',callback=exit_callback)
m2.event_subscribe(event_name='exit',callback=exit_callback)
m3.event_subscribe(event_name='exit',callback=exit_callback)

print("Running the Modules...")
m1.run()
m2.run()
m3.run()
rclpy.spin(m1) # blocks the communication with other retico modules :/
