from rclpy.node import Node
from retico_core.abstract import AbstractProducingModule, IncrementalUnit

class ROS2SubscriberModule(Node, AbstractProducingModule):
    """A ROS2 Subscriber Node
    Attributes:
        subscription: Subscription to the given ROS topic
        msg_to_iu: function which converts a ROS message of msg_type to an IncrementalUnit object
        debug(bool): if the debug mode should turned on 
    """
    def __init__(self, ros_node_name, topic_name, msg_type, msg_to_iu, debug=False,**kwargs):
        """Initializes the Ros subscriber.
        Args:
            ros_node_name(str): name of the ROS node
            topic_name(str): ROS topic from which the message is read from
            msg_type: ROS message type of the topic
            msg_to_iu: function which converts a ROS message of msg_type to an IncrementalUnit object
            debug(bool): if the debug mode should turned on 
        """
        Node.__init__(self, ros_node_name)
        AbstractProducingModule.__init__(self, **kwargs)
        
        self.subscription = self.create_subscription(msg_type, topic_name, self.ros_callback, 5)
        self.to_iu = msg_to_iu
       
        self.debug = debug
        if self.debug:
            self.get_logger().info(f"subscribing to topic: {topic_name}")

    @staticmethod
    def name():
        return "ROS2 Subscriber Node"

    @staticmethod
    def description():
        return "A Node providing reading from a ROS2 topic"

    @staticmethod
    def output_iu():
        return IncrementalUnit

    @staticmethod
    def input_ius():
        return [] 

    def process_update(self, update_message):
        pass
    
    def ros_callback(self, msg):
        if self.debug:
            self.get_logger().info(f"received {msg}!") 
        
        return self.to_iu(msg)

    def setup(self):
        pass