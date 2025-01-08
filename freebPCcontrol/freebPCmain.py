"""freebird PC flight controller"""

import sys
import pygame
import pygame.draw
import pygame.colordict
import dialog_box
from freepygame import freetext
import data_processing as dp
import time

pygame.init()
frame_number = 240
display_size = (1280, 720)
pygame.display.set_caption("freebird PC flight controller")
pygame.display.set_icon(pygame.image.load("assets\\freebird_music.ico"))
screen = pygame.display.set_mode(display_size, pygame.DOUBLEBUF | pygame.HWSURFACE)
screen.fill(pygame.colordict.THECOLORS.get("grey0"))
buffer = pygame.Surface(display_size)
clock = pygame.time.Clock()

try:
    setting_list = ["" for _ in range(0, 2)]
    setting_index = 0
    use_setting_film = True
    setting = open("setting.txt", "r")
    for set_num in setting:
        setting_list[setting_index] = set_num
        setting_index += 1
    esp32_ip = str(setting_list[0][12:len(setting_list[0]) - 2])
    print(esp32_ip)
    esp32_port = str(setting_list[1][14:len(setting_list[1]) - 2])
    print(esp32_port)
    setting.close()
except:
    dialog_box.waring_msg("setting.txt file read error!\nplease check you setting film")
    sys.exit()

receive_udp_time = 0
receive_udp_each_time = 0
receive_udp_each_time_max = 5
receive_udp_data = []
send_udp_data = []
w, a, s, d, q, e, r, f, space = 0, 0, 0, 0, 0, 0, 0, 0, 0
check = 23
state = 1
motor_1, motor_2, motor_3, motor_4 = 1000, 1000, 1000, 1000
sensor = "000000000000000000000000000000000000000000000"
motors_title_space = "            "
motors_pwm_space = "               "
sensor_title_space = "        "
event_text = freetext.SuperText(screen, [3, 5], "", "assets\\simhei.ttf", size = 12,
                                color=pygame.colordict.THECOLORS.get("grey70"))
esp32_text = freetext.SuperText(screen, [1030, 5], ("esp32_ip: " + str(esp32_ip) + "   esp32_port: " + str(esp32_port)),
                                "assets\\simhei.ttf", size = 12, color=pygame.colordict.THECOLORS.get("grey70"))

# 右边需要显示的字符
motors_pwm_title_text = freetext.SuperText(screen, [650, 35], "电机转速 (pwm)", "assets\\simhei.ttf",
                                size = 15, color = pygame.colordict.THECOLORS.get("grey70"))
motors_pwm_name_text = freetext.SuperText(screen, [655, 60], ("Motor_1" + motors_title_space + "Motor_2" +
                                motors_title_space + "Motor_3" + motors_title_space + "Motor_4"), "assets\\simhei.ttf",
                                size = 15, color = pygame.colordict.THECOLORS.get("grey70"))
motors_pwm_text = freetext.SuperText(screen, [655, 85], "", "assets\\simhei.ttf", size = 15,
                                color = pygame.colordict.THECOLORS.get("grey90"))
sensor_title_text = freetext.SuperText(screen, [650, 135], "传感器数据", "assets\\simhei.ttf", size = 15,
                                color = pygame.colordict.THECOLORS.get("grey70"))
sensor_name_text = freetext.SuperText(screen, [655, 160], ("速度 " + sensor_title_space + "高度 " + sensor_title_space +
                                "气压 " + sensor_title_space + "倾角x" + sensor_title_space + "倾角y" + sensor_title_space
                                + "倾角z"), "assets\\simhei.ttf", size = 15, color = pygame.colordict.THECOLORS.get("grey70"))
sensor_text = freetext.SuperText(screen, [655, 185], "", "assets\\simhei.ttf", size = 15,
                                color = pygame.colordict.THECOLORS.get("grey90"))
primordial_title_text = freetext.SuperText(screen, [650, 235], "原始接收数据", "assets\\simhei.ttf", size = 15,
                                color = pygame.colordict.THECOLORS.get("grey70"))
primordial_text = [freetext.SuperText(screen, [655, (680 - 20 * index)], "", "assets\\simhei.ttf",
                                size = 15, color = pygame.colordict.THECOLORS.get("grey70"))
                                for index in range(0, int((700 - 260) / 20))]  # 一共22条

# 左边需要显示的字符
controller_title_text = freetext.SuperText(screen, [10, 35], "控制面板", "assets\\simhei.ttf",
                                           size = 15, color = pygame.colordict.THECOLORS.get("grey70"))
send_title_text = freetext.SuperText(screen, [10, 235], "原始发送数据", "assets\\simhei.ttf", size = 15,
                                color = pygame.colordict.THECOLORS.get("grey70"))
send_text = [freetext.SuperText(screen, [15, (680 - 20 * index)], "", "assets\\simhei.ttf",
                                size = 15, color = pygame.colordict.THECOLORS.get("grey70"))
                                for index in range(0, int((700 - 260) / 20))]  # 一共22条

text = [event_text, esp32_text, motors_pwm_name_text, motors_pwm_title_text, motors_pwm_text, sensor_title_text,
        sensor_name_text, sensor_text, primordial_title_text, controller_title_text, send_title_text]

while True:
    event_text.set_msg("现在时间：" + str(time.localtime().tm_year) + "年 " + str(time.localtime().tm_mon) + "月 " +
                       str(time.localtime().tm_mday) + "日 " + str(time.localtime().tm_hour) + "时 " +
                       str(time.localtime().tm_min) + "分 " + str(time.localtime().tm_sec) + "秒   " +
                       "当前帧速率 " + str(int(clock.get_fps())) + "     freebird PC flight controller")

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            exit()
        elif event.type == pygame.KEYDOWN:   # 键盘事件
            if event.key == pygame.K_w:
                w = 1
            elif event.key == pygame.K_a:
                a = 1
            elif event.key == pygame.K_s:
                s = 1
            elif event.key == pygame.K_d:
                d = 1
            elif event.key == pygame.K_q:
                q = 1
            elif event.key == pygame.K_e:
                e = 1
            elif event.key == pygame.K_r:
                r = 1
            elif event.key == pygame.K_f:
                f = 1
            elif event.key == pygame.K_SPACE:
                space = 1
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_w:
                w = 0
            elif event.key == pygame.K_a:
                a = 0
            elif event.key == pygame.K_s:
                s = 0
            elif event.key == pygame.K_d:
                d = 0
            elif event.key == pygame.K_q:
                q = 0
            elif event.key == pygame.K_e:
                e = 0
            elif event.key == pygame.K_r:
                r = 0
            elif event.key == pygame.K_f:
                f = 0
            elif event.key == pygame.K_SPACE:
                space = 0
        else:
            pass

    if receive_udp_each_time == receive_udp_each_time_max:
        data, addr = dp.receive_udp_message(esp32_ip, esp32_port)
        message = dp.coding_message(23, 1, w, a, s, d, q, e, r, f, space)
        # w, a, s, d, q, e, r, f, space = 0, 0, 0, 0, 0, 0, 0, 0, 0
        print(message)
        if data == "":
            receive_udp_data.append(str(receive_udp_time) + "  Error receiving message")
            send_udp_data.append(str(receive_udp_time) + "  " + str(message))
            if len(receive_udp_data) > 22:
                receive_udp_data.pop(0)
            if len(send_udp_data) > 22:
                send_udp_data.pop(0)
        else:
            check, state, motor_1, motor_2, motor_3, motor_4, sensor = dp.decoding_message(data)
        receive_udp_each_time = 0
        receive_udp_time += 1
        dp.send_udp_message(message, esp32_ip, esp32_port)
    else:
        receive_udp_each_time += 1

    pygame.draw.aaline(screen, pygame.colordict.THECOLORS.get("grey70"), (640, 25), (640, 720), 2)
    pygame.draw.aaline(screen, pygame.colordict.THECOLORS.get("grey70"), (0, 25), (1280, 25), 2)
    pygame.draw.aaline(screen, pygame.colordict.THECOLORS.get("grey70"), (640, 120), (1280, 120), 2)
    pygame.draw.aaline(screen, pygame.colordict.THECOLORS.get("grey70"), (0, 220), (1280, 220), 2)
    motors_pwm_text.set_msg(str(motor_1) + motors_pwm_space + str(motor_2) + motors_pwm_space + str(motor_3) +
                            motors_pwm_space + str(motor_4))

    for each_text in text:
        each_text.draw()
    _primordial_text_index = 0
    for each_text in primordial_text:
        try:
            each_text.set_msg(receive_udp_data[22 - _primordial_text_index])
        except IndexError:
            each_text.set_msg(" ")
        each_text.draw()
        _primordial_text_index += 1
    _send_text_index = 0
    for each_text in send_text:
        try:
            each_text.set_msg(send_udp_data[22 - _send_text_index])
        except IndexError:
            each_text.set_msg(" ")
        each_text.draw()
        _send_text_index += 1
    pygame.display.flip()
    screen.fill(pygame.colordict.THECOLORS.get("grey0"))
    clock.tick(frame_number)
