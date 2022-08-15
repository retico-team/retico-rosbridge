from retico_core import *


class TextOutputModule(AbstractConsumingModule):
    """A module that receives text and print it in the console."""

    @staticmethod
    def name():
        return "TextOutput Module"

    @staticmethod
    def description():
        return "A module that receives text and print it in the console."

    @staticmethod
    def input_ius():
        return [text.GeneratedTextIU] 

    @staticmethod
    def output_iu():
        return None

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def process_update(self, update_message):
        for iu, ut in update_message:
            print(iu.payload)