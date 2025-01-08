"""freebird map"""
from freestate import *


class Map:
    """
    地图类

    这是一个独立的类，没有继承自任何类，也不要把祂当基类用
    提供玩家类和要放到地图中的对象类，地图会自动移动祂们

    本类中的所有方法都返回对应状态

    画面相对于地图的偏移量表示为就是画面原点的坐标
    画面原点的坐标就是（X, Y），也就是地图坐标控制画面在地图中的位置

    假设画面中有一点，画面坐标为（x, y），则地图坐标表示为（X+x, Y+y)
    判断该点是否在画面内：
        X+x > X-c, X+x < X+w+c
        Y+y > Y-c, Y+y < Y+h+c
    此时的X+x等是一个整体，在地图的层面计算
    当只需判断画面坐标时，可以忽略X和Y：
        x > -c, x < w+c
        y > -c, y < h+c

    画面本身也须满足坐标限制：
        X >= 0, X <= W-w
        Y >= 0, Y <= H-h
    """

    def __init__(self, player, objects, w, h, W, H, c):
        self.W = W  # 地图宽度
        self.H = H  # 地图高度
        self.w = w  # 画面宽度
        self.h = h  # 画面高度
        self.c = c  # 画面的边界宽度
        self.player = player  # 玩家
        self.object = objects  # 地图中的对象
        self.X = 0
        self.Y = 0

    def set_position(self, X, Y):
        state = true
        # 储存画面当前的位置，如果无法移动画面，使用此位置回退
        _X = self.X
        _Y = self.Y
        # 画面的边界检查
        if not (self.W - self.w >= self.X >= 0 and
                self.H - self.h >= self.Y >= 0):
            # 画面回到上一个位置
            self.X = _X
            self.Y = _Y
            return out_map
        else:
            self.X = X
            self.Y = Y
            return state

    def move(self, m, n):
        """
        移动画面

        本方法是以玩家为中心，移动画面
        m是须移动的横坐标
        n是须移动的纵坐标

        整个移动依次进行：
        1：画面的移动与边界检查，如过检查不通过返回False
        2：地图中对象的移动与画面对对象的边界检查，当对象在画面内时，持行对应事件
        3：玩家的移动
        """
        state = true
        # 画面移动
        self.X += m
        self.Y += n
        # 画面的边界检查
        if not (self.W - self.w >= self.X >= 0 and
                self.H - self.h >= self.Y >= 0):
            # 画面回到上一个位置
            self.X -= m
            self.Y -= n
            return out_map
        # 对象移动
        for oneself in self.object:
            oneself_x = oneself.get_x()
            oneself_y = oneself.get_y()
            oneself.set_x(oneself_x - m)
            oneself.set_y(oneself_y - n)
            oneself.set_X(oneself_x + self.X)
            oneself.set_Y(oneself_y + self.Y)
            # 如果对象在画面的判定范围内
            if (self.w + self.c > oneself_x > 0 - self.c or
                self.h + self.c > oneself_y > 0 - self.c):
                # 对象要持行的事件
                pass
        # 玩家移动
        self.player.set_X((self.X + self.w) * 0.5)  # 居中
        self.player.set_Y((self.Y + self.h) * 0.5)
        self.player.set_x(self.player.get_X() - self.X)
        self.player.set_y(self.player.get_Y() - self.Y)
        return state

    def move_player(self, m, n):
        """
        移动玩家

        本方法仅移动玩家，不移动画面
        m是须移动的横坐标
        n是须移动的纵坐标

        整个移动依次进行：
        1：移动玩家与边界检查
        2：计算玩家在画面中的位置
        """
        state = true
        # 储存玩家当前的位置
        p_X = self.player.get_X()
        p_Y = self.player.get_Y()
        # 玩家移动
        self.player.set_X(p_X + m)
        self.player.set_Y(p_Y + n)
        # 玩家的边界检查，判断是否在地图中
        if not (0 <= self.player.get_X() <= self.W and
                0 <= self.player.get_Y() <= self.H):
            # 玩家回到上一个位置
            self.player.set_X(p_X)
            self.player.set_Y(p_Y)
            return out_map
        # 计算玩家在画面中的位置
        self.player.set_x(p_X + m - self.X)
        self.player.set_y(p_Y + n - self.Y)
        # 当玩家不在画面中时
        if not (0 <= self.player.get_x() <= self.w and
                0 <= self.player.get_y() <= self.h):
            # 持行对应事件
            pass
        return state


    def update_player(self, player):
        self.player = player

    def get_player(self):
        return self.player

    def update_object(self, objects):
        self.object = objects

    def get_object(self):
        return self.object

