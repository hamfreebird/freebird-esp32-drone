#include <WiFi.h>
#include "Wire.h"
#include <string.h>
#include "FreeSixIMU.h"
#include "FIMU_ADXL345.h"
#include "FIMU_ITG3200.h"
#include "DFRobot_BMP280.h"
#include <SoftwareSerial.h>
#include <DFRobot_QMC5883.h>
#include <ESP32Servo.h>

#define  False false
#define  True true
#define  MOTOR_PIN_1 1
#define  MOTOR_PIN_2 2
#define  MOTOR_PIN_3 3
#define  MOTOR_PIN_4 4
#define  FREQ        50      // 频率
#define  CHA         0       // 通道
#define  MIN_PW      1000    // pwm的最值
#define  MAX_PW      2000
#define  RES         8       // 分辨率
#define  MIN         0       // 角度的最值
#define  MAX         180
#define  LED_BOARD   2
#define  TEST_13     7
#define  SEA_LEVEL_PRESSURE    1015.0f   // sea level pressure

// 电机位置：
//  2  1
//  3  4
Servo    motor_1;
Servo    motor_2;
Servo    motor_3;
Servo    motor_4;
typedef  DFRobot_BMP280_IIC    BMP;
BMP      bmp(&Wire, BMP::eSdoHigh);      // 气压计
DFRobot_QMC5883 compass(&Wire, /*I2C addr*/VCM5883L_ADDRESS);
WiFiUDP  UDP;
Servo    test_servo;
FreeSixIMU sixDOF = FreeSixIMU();

float    angles[3];          // 存放原始的三轴数据
int      motor_1_pwm = 0;    // 电机的初始pwm值
int      motor_2_pwm = 0;
int      motor_3_pwm = 0;
int      motor_4_pwm = 0;
int      min_degree = 0.5 / 20 * pow(2, RES);
int      max_degree = 2.5 / 20 * pow(2, RES);
char     udp_str[128];       // 存放从UDP接收的数据
int      move_speed;         // 电机调整的速度
float    altitude;           // 高度
int      fly_speed;          // 速度
int      acceleration;       // 加速度
int      angle_x;            // 三轴数据
int      angle_y;
int      angle_z;
int      angular_velocity;   // 角速度
uint32_t air_pressure;       // 气压
float    temperature;        // 温度
bool     f = False;          // 指令的状态
bool     b = False;
bool     l = False;
bool     r = False;
bool     tl = False;
bool     tr = False;
bool     u = False;
bool     d = False;
bool     a = False;
bool     bp = False;

// 函数
// 感觉直接吧代码写进循环更简洁，可以不用指针，再看吧
// show last sensor operate status
void printLastOperateStatus(BMP::eStatus_t eStatus)
{
  switch(eStatus) {
  case BMP::eStatusOK:    Serial.println("everything ok"); break;
  case BMP::eStatusErr:   Serial.println("unknow error"); break;
  case BMP::eStatusErrDeviceNotDetected:    Serial.println("device not detected"); break;
  case BMP::eStatusErrParameter:    Serial.println("parameter error"); break;
  default: Serial.println("unknow status"); break;
  }
}

void setup()
{
  pinMode(LED_BOARD, OUTPUT);
  digitalWrite(LED_BOARD, HIGH);
  
  ESP32PWM::allocateTimer(0);
  test_servo.attach(TEST_13, MIN_PW, MAX_PW);
  // test_servo.attach(motor_1, MIN_PW, MAX_PW);
  // test_servo.attach(motor_2, MIN_PW, MAX_PW);
  // test_servo.attach(motor_3, MIN_PW, MAX_PW);
  // test_servo.attach(motor_4, MIN_PW, MAX_PW);
  
  // 电机初始化
  //
  // PWM：引脚，最小脉冲宽度，最大脉冲宽度
  motor_1.attach(MOTOR_PIN_1, MIN_PW, MAX_PW);
  motor_2.attach(MOTOR_PIN_2, MIN_PW, MAX_PW);
  motor_3.attach(MOTOR_PIN_3, MIN_PW, MAX_PW);
  motor_4.attach(MOTOR_PIN_4, MIN_PW, MAX_PW);

  ledcSetup(CHA, FREQ, RES);
  ledcAttachPin(TEST_13, CHA);

  // 校正电调
  motor_1.write(MAX);
  motor_2.write(MAX);
  motor_3.write(MAX);
  motor_4.write(MAX);
  delay(2500);
  motor_1.write(motor_1_pwm);
  motor_2.write(motor_2_pwm);
  motor_3.write(motor_3_pwm);
  motor_4.write(motor_4_pwm);
  delay(500);

  // 此时可以给每个电机的pwm加个起飞时的初始值，等无人机装好后再测试，决定这个值的大小
  motor_1_pwm = 50;
  motor_2_pwm = 50;
  motor_3_pwm = 50;
  motor_4_pwm = 50;

  // 通信模块初始化
  //
  // 打开ESP32的热点，启动UDP，监听
  WiFi.softAP("ESP32_Udp_server", "");
  UDP.begin(1122);

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BOARD, OUTPUT);
  ESP32PWM::allocateTimer(0);
  test_servo.attach(TEST_13, MIN, MAX);
  WiFi.softAP("ESP32_Udp_server", "");
  UDP.begin(1122);
  Serial.begin(115200);
  bmp.reset();
  Serial.println("bmp config test");
  while(bmp.begin() != BMP::eStatusOK) {
    Serial.println("bmp begin faild");
    printLastOperateStatus(bmp.lastOperateStatus);
    delay(2000);
  }
  Serial.println("bmp begin success");
  bmp.setConfigFilter(BMP::eConfigFilter_off);        // set config filter
  bmp.setConfigTStandby(BMP::eConfigTStandby_125);    // set standby time
  bmp.setCtrlMeasSamplingTemp(BMP::eSampling_X8);     // set temperature over sampling
  bmp.setCtrlMeasSamplingPress(BMP::eSampling_X8);    // set pressure over sampling
  bmp.setCtrlMeasMode(BMP::eCtrlMeasModeNormal);     // set control measurement mode to make these settings effective

  while (!compass.begin())
  {
    Serial.println("Could not find a valid 5883 sensor, check wiring!");
    delay(50);
  }

  if(compass.isHMC())
  {
    Serial.println("Initialize HMC5883");
  }
  else if(compass.isQMC())
  {
    Serial.println("Initialize QMC5883");
  }
  else if(compass.isVCM())
  {
    Serial.println("Initialize VCM5883L");
  }
  // Wire.begin();
  // delay(5);
  // sixDOF.init();

  delay(100);
}

void loop()
{
  // 设置引脚上的PWM
  // MOTOR_1.write(motor_1_pwm);
  // MOTOR_2.write(motor_2_pwm);
  // MOTOR_3.write(motor_3_pwm);
  // MOTOR_4.write(motor_4_pwm);

  ledcWrite(CHA, min_degree);
  delay(1000);
  ledcWrite(CHA, max_degree);
  delay(1000);

  // 接收UDP数据
  // parsePacket()返回解析的数据长度，不为0则有数据
  if (UDP.parsePacket())
  {
    char val[128];
    UDP.read(val, 128);
    Serial.println(val);

    // 发送接收到的数据回去
    UDP.beginPacket(UDP.remoteIP(), UDP.remotePort());
    UDP.println(val);
    UDP.endPacket();

    // 填写数据到udp_str
    strcpy(val, udp_str);
  }

  // 气压，高度，温度数据
  float       temp = bmp.getTemperature();
  uint32_t    press = bmp.getPressure();
  float       alti = bmp.calAltitude(SEA_LEVEL_PRESSURE, press);
  air_pressure = press;
  temperature = temp;
  altitude = alti;
  Serial.println();
  Serial.println("======== start print ========");
  Serial.print("temperature (unit Celsius): "); Serial.println(temperature);
  Serial.print("pressure (unit pa):         "); Serial.println(air_pressure);
  Serial.print("altitude (unit meter):      "); Serial.println(altitude);
  Serial.println("========  end print  ========");

  // 三个轴上的倾角
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
  compass.setDeclinationAngle(declinationAngle);
  sVector_t mag = compass.readRaw();
  compass.getHeadingDegrees();
  angle_x = mag.XAxis;
  angle_y = mag.YAxis;
  angle_z = mag.ZAxis;
  Serial.print("X:");
  Serial.print(angle_x);
  Serial.print(" Y:");
  Serial.print(angle_y);
  Serial.print(" Z:");
  Serial.println(angle_z);
  Serial.print("Degress = ");
  Serial.println(mag.HeadingDegress);
  // sixDOF.getEuler(angles);
  // Serial.print(angles[0]);
  // Serial.print(" | ");
  // Serial.print(angles[1]);
  // Serial.print(" | ");
  // Serial.println(angles[2]);

  // 函数返回后应该将所有状态重新设为False
  if (a == False)
  {
    // back_pos
    f = False;
    b = False;
    l = False;
    r = False;
    tl = False;
    tr = False;
    u = False;
    d = False;
  }

  // 电机控制
  // 64位的数组，前两位是校验，第三位状态
  // 4-12对应九种操作:f,b,l,r,tl,tr,u,d,a
  if (udp_str[3] == 1)
  {
    motor_1_pwm = motor_1_pwm - move_speed;
    motor_2_pwm = motor_2_pwm - move_speed;
    f = True;
  } 
  else if (udp_str[4] == 1) 
  {
    motor_3_pwm = motor_3_pwm - move_speed;
    motor_4_pwm = motor_4_pwm - move_speed;
    b = True;
  } 
  else if (udp_str[5] == 1) 
  {
    motor_2_pwm = motor_2_pwm - move_speed;
    motor_3_pwm = motor_3_pwm - move_speed;
    l = True;
  } 
  else if (udp_str[6] == 1) 
  {
    motor_1_pwm = motor_1_pwm - move_speed;
    motor_4_pwm = motor_4_pwm - move_speed;
    r = True;
  } 
  else if (udp_str[7] == 1) 
  {
    motor_1_pwm = motor_1_pwm - move_speed;
    motor_3_pwm = motor_3_pwm - move_speed;
    tl = True;
  } 
  else if (udp_str[8] == 1) 
  {
    motor_2_pwm = motor_2_pwm - move_speed;
    motor_4_pwm = motor_4_pwm - move_speed;
    tr = True;
  } 
  else if (udp_str[9] == 1) 
  {
    motor_1_pwm = motor_1_pwm + move_speed;
    motor_2_pwm = motor_2_pwm + move_speed;
    motor_3_pwm = motor_3_pwm + move_speed;
    motor_4_pwm = motor_4_pwm + move_speed;
    u = True;
  } 
  else if (udp_str[10] == 1) 
  {
    motor_1_pwm = motor_1_pwm - move_speed;
    motor_2_pwm = motor_2_pwm - move_speed;
    motor_3_pwm = motor_3_pwm - move_speed;
    motor_4_pwm = motor_4_pwm - move_speed;
    d = True;
  }

  delay(10);

}

// 所有的操作都没有根据加速度计自动调整的功能，等硬件到了在写
// 加速度计的计算流程：
// 1 每一次循环读取加速度计的数据
// 2 在所有操作之前，向加速方向的反方向计算pwm，然后在进行操作
//   如果是持续的上一次循环的操作，要保证在该方向上的加速度为0
// 3 如果是自动降落，还要单独考虑重力方向的速度

// back_position就是加速度的计算部分
// altitude;
// fly_speed;
// acceleration;
// angle_x;
// angle_y;
// angle_z;
// angular_velocity;
// air_pressure;

// 需要逐个判断每个方向上是否需要计算pwm

// 自动降落，此时这个函数应该取代back_position的功能

// 还应该有cmt2300A的代码
