import pandas as pd
import os
class DataLoader:
    timestamp = None
    def __init__(self, path):
        self.current_image = None
        self.timestamp = None
        self.path = path
    def load_next_image(self, line):
        line = line.split(',')
        self.timestamp = line[0]
        self.current_image = os.path.join(self.path, line[1].split('\n')[0])
    
        