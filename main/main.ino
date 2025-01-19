#include <Servo.h>
#include <WiFi.h>
#define False false
#define True true

// 电机部分
//
// 定义电机引脚
#define MOTOR_PIN_1 1
#define MOTOR_PIN_2 2
#define MOTOR_PIN_3 3
#define MOTOR_PIN_4 4

// 电机位置：
//  2  1
//  3  4

// 定义PWM相关常量
#define FREQ    50    // 频率
#define MIN_PW  1000
#define MAX_PW  2000
#define RES     8     // 分辨率
#define MIN     0
#define MAX     180

// 定义PWM值
int motor_1_pwm = 0;
int motor_2_pwm = 0;
int motor_3_pwm = 0;
int motor_4_pwm = 0;
int *p_motor_1_pwm, *p_motor_2_pwm, *p_motor_3_pwm, *p_motor_4_pwm;
p_motor_1_pwm = motor_1_pwm;
p_motor_2_pwm = motor_2_pwm;
p_motor_3_pwm = motor_3_pwm;
p_motor_4_pwm = motor_4_pwm;

// 定义PWM对象
Servo MTOTR_1
Servo MTOTR_2
Servo MTOTR_3
Servo MTOTR_4

// 通信模块部分
//
// 声明UDP对象，开启服务端，监听（ip, 端口）
WiFiUDP UDP;

// 存放UDP接收的数据
char udp_str[128];

// 飞行参数
int move_speed;

// 传感器数据，硬件到了再改
int altitude;
int fly_speed;
int acceleration;
int angle_x;
int angle_y;
int angle_z;
int angular_velocity;
int air_pressure;

// 当前的操作（十种操作:f,b,l,r,tl,tr,u,d,a,bp）
bool f = False;
bool b = False;
bool l = False;
bool r = False;
bool tl = False;
bool tr = False;
bool u = False;
bool d = False;
bool a = False;
bool bp = False;

// 函数
// 感觉直接吧代码写进循环更简洁，可以不用指针，再看吧
// int get_rx_date();
// void send_tx_date(int tx_date);
void go_forward(int d_motor_1, int d_motor_2, int d_move_speed, int *p_motor_1_pwm, int *p_motor_2_pwm);
void go_back(int d_motor_3, int d_motor_4, int d_move_speed, int *p_motor_3_pwm, int *p_motor_4_pwm);
void go_left(int d_motor_2, int d_motor_3, int d_move_speed, int *p_motor_2_pwm, int *p_motor_3_pwm);
void go_right(int d_motor_1, int d_motor_4, int d_move_speed, int *p_motor_1_pwm, int *p_motor_4_pwm);
void turn_left(int d_motor_1, int d_motor_3, int d_move_speed, int *p_motor_1_pwm, int *p_motor_3_pwm);
void turn_right(int d_motor_2, int d_motor_4, int d_move_speed, int *p_motor_2_pwm, int *p_motor_4_pwm);
void go_up(int d_motor_1, int d_motor_2, int d_motor_3, int d_motor_4, int d_move_speed,
           int *p_motor_1_pwm, int *p_motor_2_pwm, int *p_motor_3_pwm, int *p_motor_4_pwm);
void go_down(int d_motor_1, int d_motor_2, int d_motor_3, int d_motor_4, int d_move_speed,
             int *p_motor_1_pwm, int *p_motor_2_pwm, int *p_motor_3_pwm, int *p_motor_4_pwm);
void back_positive(int d_motor_1, int d_motor_2, int d_motor_3, int d_motor_4, bool b_motor_1, bool b_motor_2, bool b_motor_3, bool b_motor_4,
                   int *p_motor_1_pwm, int *p_motor_2_pwm, int *p_motor_3_pwm, int *p_motor_4_pwm,
                   bool b_f, bool b_b, bool b_l, bool b_r, bool b_tl, bool b_tr, bool b_u, bool b_d);
void automatic_landing(int d_motor_1, int d_motor_2, int d_motor_3, int d_motor_4, int d_move_speed,
                       int *p_motor_1_pwm, int *p_motor_2_pwm, int *p_motor_3_pwm, int *p_motor_4_pwm,
                       bool b_f, bool b_b, bool b_l, bool b_r, bool b_tl, bool b_tr, bool b_u, bool b_d);

void setup()
{
  // 电机初始化
  //
  // PWM：引脚，最小脉冲宽度，最大脉冲宽度
  MOTOR_1.attach(MOTOR_PIN_1, MIN_PW, MAX_PW);
  MOTOR_2.attach(MOTOR_PIN_2, MIN_PW, MAX_PW);
  MOTOR_3.attach(MOTOR_PIN_3, MIN_PW, MAX_PW);
  MOTOR_4.attach(MOTOR_PIN_4, MIN_PW, MAX_PW);

  // 校正电调
  MOTOR_1.write(MAX);
  MOTOR_2.write(MAX);
  MOTOR_3.write(MAX);
  MOTOR_4.write(MAX);
  delay(2500);
  MOTOR_1.write(motor_1_pwm);
  MOTOR_2.write(motor_2_pwm);
  MOTOR_3.write(motor_3_pwm);
  MOTOR_4.write(motor_4_pwm);

  // 此时可以给每个电机的pwm加个起飞时的初始值，等无人机装好后再测试，决定这个值的大小

  // 通信模块初始化
  //
  // 打开ESP32的热点，启动UDP，监听
  WiFi.softAP("ESP32_Udp_server", "");
  UDP.begin(1122);

}

void loop()
{
  // 设置引脚上的PWM
  MOTOR_1.write(motor_1_pwm);
  MOTOR_2.write(motor_2_pwm);
  MOTOR_3.write(motor_3_pwm);
  MOTOR_4.write(motor_4_pwm);

  // 接收UDP数据
  // parsePacket()返回解析的数据长度，不为0则有数据
  if (UDP.parsePacket())
  {
    char val[128];
    UDP.read(val, 128);
    Serial.println(val):

    // 发送接收到的数据回去
    UDP.beginPacket(UDP.remoteIP(), UDP.remotePort());
    UDP.println(val);
    UDP.endPacket();

    // 填写数据到udp_str
    udp_str = val;
  }

  // 函数返回后应该将所有状态重新设为False
  if (a == False)
    back_positive(motor_1_pwm, motor_2_pwm, motor_3_pwm, motor_4_pwm, true, true, true, true,
                  *p_motor_1_pwm, *p_motor_2_pwm, *p_motor_3_pwm, *p_motor_4_pwm, f, b, l, r, tl ,tr ,u ,d);
    f = False;
    b = False;
    l = False;
    r = False;
    tl = False;
    tr = False;
    u = False;
    d = False;

  // 电机控制
  // 64位的数组，前两位是校验，第三位状态
  // 4-12对应九种操作:f,b,l,r,tl,tr,u,d,a
  if (udp_str[3] == 1)
  {
    go_forward(motor_1_pwm, motor_2_pwm, fly_speed, *p_motor_1_pwm, *p_motor_2_pwm);
    f = True;
  } 
  else if (udp_str[4] == 1) 
  {
    go_back(motor_3_pwm, motor_4_pwm, fly_speed, *p_motor_3_pwm, *p_motor_4_pwm);
    b = True;
  } 
  else if (udp_str[5] == 1) 
  {
    go_left(motor_2_pwm, motor_3_pwm, fly_speed, *p_motor_2_pwm, *p_motor_3_pwm);
    l = True;
  } 
  else if (udp_str[6] == 1) 
  {
    go_right(motor_1_pwm, motor_4_pwm, fly_speed, *p_motor_1_pwm, *p_motor_4_pwm);
    r = True;
  } 
  else if (udp_str[7] == 1) 
  {
    turn_left(motor_1_pwm, motor_3_pwm, fly_speed, *p_motor_1_pwm, *p_motor_3_pwm);
    tl = True;
  } 
  else if (udp_str[8] == 1) 
  {
    turn_right(motor_2_pwm, motor_4_pwm, fly_speed, *p_motor_2_pwm, *p_motor_4_pwm);
    tr = True;
  } 
  else if (udp_str[9] == 1) 
  {
    go_up(motor_1_pwm, motor_2_pwm, motor_3_pwm, motor_4_pwm, fly_speed, *p_motor_1_pwm, *p_motor_2_pwm, *p_motor_3_pwm, *p_motor_4_pwm);
    u = True;
  } 
  else if (udp_str[10] == 1) 
  {
    go_down(motor_1_pwm, motor_2_pwm, motor_3_pwm, motor_4_pwm, fly_speed, *p_motor_1_pwm, *p_motor_2_pwm, *p_motor_3_pwm, *p_motor_4_pwm);
    d = True;
  }

}

// 所有的操作都没有根据加速度计自动调整的功能，等硬件到了在写
// 加速度计的计算流程：
// 1 每一次循环读取加速度计的数据
// 2 在所有操作之前，向加速方向的反方向计算pwm，然后在进行操作
//   如果是持续的上一次循环的操作，要保证在该方向上的加速度为0
// 3 如果是自动降落，还要单独考虑重力方向的速度

void go_forward(int d_motor_1, int d_motor_2, int d_move_speed, int *p_motor_1_pwm, int *p_motor_2_pwm)
{
  *p_motor_1_pwm = d_motor_1 - d_move_speed;
  *p_motor_2_pwm = d_motor_2 - d_move_speed;
}

void go_back(int d_motor_3, int d_motor_4, int d_move_speed, int *p_motor_3_pwm, int *p_motor_4_pwm)
{
  *p_motor_3_pwm = d_motor_3 - d_move_speed;
  *p_motor_4_pwm = d_motor_4 - d_move_speed;
}

void go_left(int d_motor_2, int d_motor_3, int d_move_speed, int *p_motor_2_pwm, int *p_motor_3_pwm)
{
  *p_motor_2_pwm = d_motor_2 - d_move_speed;
  *p_motor_3_pwm = d_motor_3 - d_move_speed;
}

void go_right(int d_motor_1, int d_motor_4, int d_move_speed, int *p_motor_1_pwm, int *p_motor_4_pwm)
{
  *p_motor_1_pwm = d_motor_1 - d_move_speed;
  *p_motor_4_pwm = d_motor_4 - d_move_speed;
}

void turn_left(int d_motor_1, int d_motor_3, int d_move_speed, int *p_motor_1_pwm, int *p_motor_3_pwm)
{
  *p_motor_1_pwm = d_motor_1 - d_move_speed;
  *p_motor_3_pwm = d_motor_3 - d_move_speed;
}

void turn_right(int d_motor_2, int d_motor_4, int d_move_speed, int *p_motor_2_pwm, int *p_motor_4_pwm)
{
  *p_motor_2_pwm = d_motor_2 - d_move_speed;
  *p_motor_4_pwm = d_motor_4 - d_move_speed;
}

void go_up(int d_motor_1, int d_motor_2, int d_motor_3, int d_motor_4, int d_move_speed, int *p_motor_1_pwm, int *p_motor_2_pwm, int *p_motor_3_pwm, int *p_motor_4_pwm)
{
  *p_motor_1_pwm = d_motor_1 + d_move_speed;
  *p_motor_2_pwm = d_motor_2 + d_move_speed;
  *p_motor_3_pwm = d_motor_3 + d_move_speed;
  *p_motor_4_pwm = d_motor_4 + d_move_speed;
}

void go_down(int d_motor_1, int d_motor_2, int d_motor_3, int d_motor_4, int d_move_speed, int *p_motor_1_pwm, int *p_motor_2_pwm, int *p_motor_3_pwm, int *p_motor_4_pwm)
{
  *p_motor_1_pwm = d_motor_1 - d_move_speed;
  *p_motor_2_pwm = d_motor_2 - d_move_speed;
  *p_motor_3_pwm = d_motor_3 - d_move_speed;
  *p_motor_4_pwm = d_motor_4 - d_move_speed;
}

// back_position就是加速度的计算部分
// altitude;
// fly_speed;
// acceleration;
// angle_x;
// angle_y;
// angle_z;
// angular_velocity;
// air_pressure;

void back_positive(int d_motor_1, int d_motor_2, int d_motor_3, int d_motor_4, bool b_motor_1, bool b_motor_2,
                   bool b_motor_3, bool b_motor_4, int *p_motor_1_pwm, int *p_motor_2_pwm, int *p_motor_3_pwm, int *p_motor_4_pwm,
                   bool b_f, bool b_b, bool b_l, bool b_r, bool b_tl, bool b_tr, bool b_u, bool b_d)
{
  // 需要逐个判断每个方向上是否需要计算pwm
}

// 自动降落，此时这个函数应该取代back_position的功能

void automatic_landing(int d_motor_1, int d_motor_2, int d_motor_3, int d_motor_4, int d_move_speed,
                       int *p_motor_1_pwm, int *p_motor_2_pwm, int *p_motor_3_pwm, int *p_motor_4_pwm,
                       bool b_f, bool b_b, bool b_l, bool b_r, bool b_tl, bool b_tr, bool b_u, bool b_d)
{
  
}

// 还应该有cmt2300A的代码
