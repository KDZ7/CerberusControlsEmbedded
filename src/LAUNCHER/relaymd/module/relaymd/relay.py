import os
import yaml
from launch.actions import OpaqueFunction

class Opaque:
    def __init__(self, function):
        self.function = function
    def __call__(self, *args, **kwargs):
        return OpaqueFunction(function=self.function, args=args, kwargs=kwargs)
    
class Relay:
    dict = None
    
    def __init__(self):
        self.dict = None
    
    def __call__(self, *args, **kwargs):
        return Relay.action(*args, **kwargs)
    
    @staticmethod
    @Opaque
    def read(context, input):
        input = input.perform(context)
        if os.path.isfile(input):
            with open(input, "r", encoding="utf-8") as file:
                Relay.dict = yaml.safe_load(file)
        else:
            Relay.dict = yaml.safe_load(input)
            
    @staticmethod
    @Opaque
    def action(context, function, *args, **kwargs):
        return function(context, *args, **kwargs)
    
    def __getitem__(self, key):
        self.dict = Relay.dict
        return self.dict.get(key, None)
    
    def __setitem__(self, key, value):
        Relay.dict[key] = value

class Helpers:
    @staticmethod
    def load_yaml(file):
        with open(file, "r", encoding="utf-8") as file:
            return yaml.safe_load(file)