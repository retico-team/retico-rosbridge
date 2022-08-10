from rclpy.node import Node
from retico_core.abstract import AbstractConsumingModule, IncrementalUnit

class ROS2PublisherModule(Node, AbstractConsumingModule):
    """A ROS Publishing Node.
    Attributes:
        publisher(Publisher): ROS Publisher which publishes to the given topic
        msg_type: ROS message type of the topic
        iu_to_msg: function which converts an object of any type into a message of msg_type
        debug(bool): if the debug mode should turned on 
        exit_callback: callback function when the exit command is received
    """
    def __init__(self, ros_node_name, topic_name, msg_type, iu_to_msg, debug=False, **kwargs):
        """Constructor of ROS2PublisherModule.
        Args: 
            ros_node_name(str): name of the ROS node
            topic_name(str): the topic where the information will be published to
            msg_type: ROS message type of the topic
            iu_to_msg: function which converts an IncrementalUnit object into a ROS message of msg_type
            debug(bool): if the debug mode should turned on 
        """
        Node.__init__(self, ros_node_name)
        AbstractConsumingModule.__init__(self, **kwargs)        
        
        self.publisher = self.create_publisher(msg_type, topic_name, 1)
        self.msg_type = msg_type
        self.to_msg = iu_to_msg
        
        self.debug = debug
        if self.debug: 
            self.get_logger().info(f"created publisher on topic: {topic_name}")
    
    @staticmethod
    def name():
        return "ROS2 Publisher Node"

    @staticmethod
    def description():
        return "A Node providing publisher to a ROS2 topic"

    @staticmethod
    def input_ius():
        return [IncrementalUnit] 

    @staticmethod
    def output_iu():
        return None
        #return IncrementalUnit

    def process_update(self, update_message):
        if self.debug:
            print("ROS2Publisher recieved a new IU!")
        for iu, ut in update_message:
            if iu.payload is None:
                self.exit_callback('ROS2PublisherModule', 'exit')
            else:
                msg = self.to_msg(iu)
                self.publisher.publish(msg)
                if self.debug:
                    self.get_logger().info(f"publishing: {msg}")

    def setup(self):
        pass

    def event_subscribe(self, event_name, callback):
        if event_name == "exit":
            self.exit_callback = callback
        else:
            print('Not supported subscription for the event:',event_name)