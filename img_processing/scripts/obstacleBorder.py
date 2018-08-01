import math
from geometry import *

class Border(Point):
    def __init__(self ,left_top_point ,right_top_point , left_bot_point ,right_bot_point):
        self.left_top_point = left_top_point
        self.right_top_point = right_top_point
        self.left_bot_point = left_bot_point
        self.right_bot_point = right_bot_point

    def print_points(self):
        print((self.left_top_point.x , self.left_top_point.y) , (self.right_top_point.x , self.right_top_point.y)
        , (self.left_bot_point.x , self.left_bot_point.y) , (self.right_bot_point.x , self.right_bot_point.y))
    def get_left_top(self):
        return self.left_top_point
    def get_right_top(self):
        return self.right_top_point
    def get_left_bot(self):
        return self.left_bot_point
    def get_right_bot(self):
        return self.right_bot_point                
