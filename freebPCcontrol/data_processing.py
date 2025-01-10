"""data processing"""

import socket


def coding_message(check, state, w, a, s, d, q, e, r, f, space):
    """
    64位的数组，前两位是校验，第三位状态
    4-12对应九种操作
    """
    _message = (str(check) + str(state) + str(w) + str(a) + str(s) + str(d) + str(q) + str(e) +
                str(r) + str(f) + str(space) + "0000000000000000000000000000000000000000000000000000")
    return _message

def decoding_message(_message):
    """
    解码64位的数据，前两位是校验，第三位状态
    4-7， 8-11，12-15，16-19 是电机pwm值，后面都是传感器值
    """
    _check = _message[0:1]
    _state = _message[2]
    _motor_1, _motor_2, _motor_3, _motor_4 = _message[3:6], _message[7:10], _message[11:14], _message[15:18]
    _sensor = _message[19:63]
    return _check, _state, _motor_1, _motor_2, _motor_3, _motor_4, _sensor

def send_udp_message(message, ip, port):
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        sock.sendto(message.encode("utf-8"), (ip, port))
    except Exception as e:
        print(f"Error sending message: {e}")
    finally:
        sock.close()

def receive_udp_message(ip, port, buffer_size=128):
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        # Bind the socket to the port
        sock.bind((ip, port))
        data, addr = sock.recvfrom(buffer_size)
        message = data.decode()
    except Exception as e:
        print(f"Error receiving message: {e}")
        message, addr = "", ""
    finally:
        sock.close()

    return message, addr

