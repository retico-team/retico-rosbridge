import rclpy
import sys
sys.path.append("..")
from retico_textinput import TextInput
from retico_ros2.text_to_joint_angles import TextToJointAnglesModule
from retico_ros2.ros2_publisher import ROS2PublisherModule
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def to_ur3e_joint_angles(iu)-> JointTrajectory:
    """ turns an ReTiCo IncrementalUnit object into a ROS2 JointTrajectory message."""
    joint_angles = iu.payload
    traj = JointTrajectory()
    traj.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        
    point = JointTrajectoryPoint()
    point.positions = joint_angles # list of floats
    point.time_from_start = Duration(sec=4)

    traj.points.append(point)
    
    return traj

print("Instantiating Modules...")
m1 = TextInput()
m2 = TextToJointAnglesModule()

rclpy.init()
m3 = ROS2PublisherModule(ros_node_name='publisher_joint_trajectory_controller',
                        topic_name="/joint_trajectory_controller/joint_trajectory", 
                        msg_type=JointTrajectory,
                        iu_to_msg=to_ur3e_joint_angles,
                        )

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