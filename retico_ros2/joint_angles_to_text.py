import os
import numpy as np
from retico_core import *


class JointAnglesToTextModule(AbstractModule):
    """A Module that converts joint values to text."""
    def __init__(self, debug=False, **kwargs):
        super().__init__(**kwargs)
        self.joint_angles = None
        self.callback = None
        self.debug = debug
    
    @staticmethod
    def name():
        return "Joint Angles To Text Module"

    @staticmethod
    def description():
        return "A Module that generates joint angles from text command."

    @staticmethod
    def input_ius():
        return [IncrementalUnit] # list of floats

    @staticmethod
    def output_iu():
       return text.GeneratedTextIU 

    def setup(self):
        pass 
            
    def process_update(self, update_message):
        if self.debug:
            print("[JointAnglesToTextModule] new message received!")
        for iu, ut in update_message:
            joint_state = iu.payload # list of floats
            output_iu = self.create_iu()
            print(joint_state)
            
            if (joint_state == np.array([1.57, -1.57, 0.0, -1.57, 0.0 ,0.0])).all():
                output_iu.payload = "Yayy, I am moving forward!"
            elif (joint_state == np.array([-1.57, -1.57, 0.0, -1.57, 0.0 ,0.0])).all():
                output_iu.payload = "Watch out, I am moving backward!"
            elif (joint_state == np.array([0.0, -1.57, -0.785, -1.57, 0.0 ,0.0])).all():
                output_iu.payload = "Yes! Moving to the left!"
            elif (joint_state == np.array([0.0, -1.57, 0.785, -1.57, 0.0 ,0.0])).all():
                output_iu.payload = "I am always right! B-)"
            elif (joint_state == np.array([0.0, -1.57, 0.0, -1.57, 0.0 ,0.0])).all():
                output_iu.payload = "Ok, I will raise my head!"
            elif (joint_state == np.array([1.57, -1.57, 2.355, -1.57, 0.0 ,0.0])).all():
                output_iu.payload = "You can't see me anymore, cause I am moving down!"
            else:
                output_iu.payload = "I don't understand what you mean!"
                
        
        return UpdateMessage.from_iu(output_iu, UpdateType.ADD)

    def event_subscribe(self, event_name, callback):
        if event_name == "exit":
            self.callback = callback
        else:
            print('Not supported subscription for the event:',event_name)

    def shutdown(self):
        pass