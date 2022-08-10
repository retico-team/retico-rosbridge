"""original code from Kranti, modified by Opal"""
from retico_core import *


class TextInput(AbstractProducingModule):
    """A module that produces IUs containing text that are received from keyboard."""

    @staticmethod
    def name():
        return "TextInput Module"

    @staticmethod
    def description():
        return "A producing module that receives input text from keyboard."

    @staticmethod
    def output_iu():
        return text.GeneratedTextIU

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.text = None

    def process_update(self, update_message):
        output_iu = self.create_iu()
        output_iu.payload = self.get_text() 
        output_iu.dispatch = True
        return UpdateMessage.from_iu(output_iu, UpdateType.ADD)

    def set_text(self, text):
        self.text = text

    def get_text(self):
        print('Type something:')
        return input('>> ')