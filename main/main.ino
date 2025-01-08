#include <Servo.h>
#include <WiFi.h>

// 电机部分
//
// 定义电机引脚
#define MOTOR_PIN_1 1
#define MOTOR_PIN_2 2
#define MOTOR_PIN_3 3
#define MOTOR_PIN_4 4

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
char udp_str;

int get_rx_date();
void send_tx_date(int tx_date);

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

  // 通信模块初始化
  //
  // 打开ESP32的热点，启动UDP，监听
  WiFi.softAP("ESP32_Udp_server", "");
  UDP.begin(1122);
  Serial.begin(115200);

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
    char val = UDP.read();
    Serial.println(val):

    // 发送接收到的数据回去
    UDP.beginPacket(UDP.remoteIP(), UDP.remotePort());
    UDP.println(val);
    UDP.endPacket();

    // 填写数据到udp_str
    udp_str = val;
  }

}

int get_rx_date()
{
  return 0;
}

void send_tx_date(int tx_date)
{
  
}
