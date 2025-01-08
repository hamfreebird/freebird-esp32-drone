import freebasicobject
from freestate import *


class Object(freebasicobject.BasicObject):

    def __init__(self, X, Y, c, picture):
        super().__init__(X, Y, 0, 0)
        self.c = c  # 边界
        self.picture = picture  # 用于绘制的对象图片

    def draw(self):
        pass


class Player(freebasicobject.BasicObject):

    def __init__(self, X, Y, x, y, c, picture):
        super().__init__(X, Y, x, y)
        self.c = c  # 边界
        self.picture = picture  # 用于绘制的角色图片

    def draw(self):
        pass



