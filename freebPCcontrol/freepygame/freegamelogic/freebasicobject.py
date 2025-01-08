from freestate import *

class BasicObject:
    """所有对象的基类，仅提供坐标的读写"""

    def __init__(self, X, Y, x, y):
        self.X = X
        self.Y = Y
        self.x = x
        self.y = y

    def get_X(self):
        return self.X

    def get_Y(self):
        return self.Y

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def set_X(self, X):
        self.X = X

    def set_Y(self, Y):
        self.Y = Y

    def set_x(self, x):
        self.x = x

    def set_y(self, y):
        self.y = y

