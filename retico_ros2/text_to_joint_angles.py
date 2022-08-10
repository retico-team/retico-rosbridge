import os
import json
from retico_core import *

class InvalidCommandException(Exception):
    pass

class TextToJointAnglesModule(AbstractModule):
    """A Module that generates joint values from text."""
    def __init__(self, debug=False, **kwargs):
        super().__init__(**kwargs)
        self.joint_angles = None
        self.callback = None
        self.debug = debug
    
    @staticmethod
    def name():
        return "Text To Joint Angles Module"

    @staticmethod
    def description():
        return "A Module that generates joint angles from text command."

    @staticmethod
    def input_ius():
        return [text.GeneratedTextIU] 

    @staticmethod
    def output_iu():
       return IncrementalUnit # list of floats

    def setup(self):
        # hard code!
        with open('../retico_ros2/config/joint_angles.json', 'r') as f_in:
            self.joint_angles = json.load(f_in) 
            
    def process_update(self, update_message):
        if self.debug:
            print("[TextToJointAnglesModule] new message received!")
        try:
            for iu, ut in update_message:
                iu_text = iu.payload
                output_iu = self.create_iu()
                
                if 'left' in iu_text:
                    output_iu.payload = self.joint_angles['left']
                elif 'right' in iu_text:
                    output_iu.payload = self.joint_angles['right']
                elif 'up' in iu_text:
                    output_iu.payload = self.joint_angles['up']
                elif 'down' in iu_text:
                    output_iu.payload = self.joint_angles['down']
                elif 'forward' in iu_text:
                    output_iu.payload = self.joint_angles['forward']
                elif 'back' in iu_text:
                    output_iu.payload = self.joint_angles['backward']
                elif 'stop' in iu_text:
                    self.callback('TextToJointAnglesModule', 'exit')
                else: 
                    raise InvalidCommandException()
    
        except InvalidCommandException:
            print("Hmm... I don't know how to move! Let me get back to the default position.")
            output_iu.payload = self.joint_angles['default']
            
        finally:
            return UpdateMessage.from_iu(output_iu, UpdateType.ADD)


    def event_subscribe(self, event_name, callback):
        if event_name == "exit":
            self.callback = callback
        else:
            print('Not supported subscription for the event:',event_name)

    def shutdown(self):
        pass