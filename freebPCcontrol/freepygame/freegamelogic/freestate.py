# 对象的状态，按范围或优先级从小到大
# 总状态：0-1
true = 0   # True
false = 1  # False
# 对象的位置：2-199
in_screen = 10
out_screen = 20
in_map = 100
at_left_of_map = 104  # 在地图的不同边界
at_top_of_map = 105
at_right_of_map = 106
at_bottom_of_map = 107
out_map = 110
# 对象的事件：200-499
# 用户自定义状态为500-999
# 地图专有状态: 1000-1099
map_smaller_than_screen = 1010
# 其他状态：>=2000
