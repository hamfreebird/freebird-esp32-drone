#include "DFRobot_BMP280.h"
#include <DFRobot_QMC5883.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include "Wire.h"
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

typedef DFRobot_BMP280_IIC    BMP;    // ******** use abbreviations instead of full names ********
BMP   bmp(&Wire, BMP::eSdoHigh);
#define SEA_LEVEL_PRESSURE    1015.0f   // sea level pressure
DFRobot_QMC5883 compass(&Wire, /*I2C addr*/VCM5883L_ADDRESS);
float angles[3]; // yaw pitch roll
// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();

// 上一次的角度值和时间戳
float last_angles[3] = {0, 0, 0};  // 存储上一次的roll, pitch, yaw角度值
uint32_t last_angle_time = 0;       // 存储上一次角度读取的时间戳

// 定义传感器数据结构
typedef struct {
    float accel[3];      // 加速度计数据 (m/s^2) [x, y, z]
    float gyro[3];       // 陀螺仪数据 (rad/s) [x, y, z]
    float baro;          // 气压计数据 (Pa)
    float temp;          // 温度计数据 (°C)
    float alti;          // 高度数据 (m)
    uint32_t timestamp;  // 时间戳 (ms)
    bool valid;          // 数据有效性标志
} sensor_data_t;

// 定义PID控制器结构
typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float integral;     // 积分项
    float prev_error;   // 上一次误差
    float prev_measurement; // 上一次测量值
    float output;       // 输出值
    float output_limit; // 输出限制
    float integral_limit; // 积分项限制
    float deadband;     // 死区
} pid_controller_t;

// 定义四元数结构
typedef struct {
    float w, x, y, z;
} quaternion_t;

// 定义欧拉角结构
typedef struct {
    float roll;         // 横滚角 (rad)
    float pitch;        // 俯仰角 (rad)
    float yaw;          // 偏航角 (rad)
} euler_angles_t;

// 定义无人机状态
typedef struct {
    quaternion_t attitude;  // 当前姿态(四元数)
    euler_angles_t euler;   // 当前姿态(欧拉角)
    float altitude;         // 当前高度(m)
    float velocity[3];      // 当前速度(m/s) [x, y, z]
    float position[3];      // 当前位置(m) [x, y, z]
    uint32_t timestamp;     // 状态时间戳
} drone_state_t;

// 定义控制输出
typedef struct {
    float motor1;       // 电机1 PWM值 (0-100%)
    float motor2;       // 电机2 PWM值 (0-100%)
    float motor3;       // 电机3 PWM值 (0-100%)
    float motor4;       // 电机4 PWM值 (0-100%)
    bool armed;         // 电机使能标志
} control_output_t;

// 定义无人机配置
typedef struct {
    float mass;             // 无人机质量 (kg)
    float motor_min_pwm;    // 电机最小PWM值 (%)
    float motor_max_pwm;    // 电机最大PWM值 (%)
    float hover_throttle;   // 悬停油门值 (%)
    euler_angles_t max_angle; // 最大允许角度 (rad)
    float max_altitude;     // 最大允许高度 (m)
} drone_config_t;

// 定义UDP数据包结构
typedef struct {
    uint32_t sequence;      // 数据包序列号
    uint32_t timestamp;     // 时间戳
    euler_angles_t attitude; // 姿态角
    float altitude;         // 高度
    float velocity[3];      // 速度
    float motor_output[4];  // 电机输出
    float pid_output[5];    // PID输出 [roll, pitch, yaw, alt, vel_z]
    uint8_t status;         // 状态标志位
} udp_data_packet_t;

// 全局变量
sensor_data_t sensor_data;
drone_state_t drone_state;
control_output_t control_output;
drone_config_t drone_config;

// PID控制器实例
pid_controller_t roll_pid = {3.0, 0.8, 0.2, 0, 0, 0, 0, 45.0, 25.0, 0.05};
pid_controller_t pitch_pid = {3.0, 0.8, 0.2, 0, 0, 0, 0, 45.0, 25.0, 0.05};
pid_controller_t yaw_pid = {2.0, 0.5, 0.1, 0, 0, 0, 0, 45.0, 25.0, 0.1};
pid_controller_t alt_pid = {1.5, 0.4, 0.5, 0, 0, 0, 0, 100.0, 50.0, 0.2};
pid_controller_t vel_z_pid = {1.2, 0.3, 0.2, 0, 0, 0, 0, 50.0, 25.0, 0.1};

// 目标状态
euler_angles_t target_attitude = {0, 0, 0};
float target_altitude = 5.0;        // 目标高度5米
float target_velocity_z = 0.0;      // 目标垂直速度

// 卡尔曼滤波器结构（简化版）
typedef struct {
    float x;        // 状态估计值
    float p;        // 估计误差协方差
    float q;        // 过程噪声协方差
    float r;        // 测量噪声协方差
    float k;        // 卡尔曼增益
} kalman_filter_t;

// 高度估计卡尔曼滤波器
kalman_filter_t alt_kalman = {0, 1, 0.01, 0.1, 0};
kalman_filter_t vel_z_kalman = {0, 1, 0.01, 0.2, 0};

// UDP通信相关变量
WiFiUDP udp;
const char* udpAddress = "192.168.1.100";  // 目标IP地址
const uint16_t udpPort = 12345;            // 目标端口
uint32_t udpSequence = 0;                  // UDP数据包序列号
uint32_t controlCycleCount = 0;            // 控制周期计数器
const uint32_t udpSendInterval = 10;       // 每10个控制周期发送一次数据
bool udpStartSend = false;                  // 是否开启udp

// WiFi配置
const char* ssid = "602";
const char* password = "freebirdflyinthesky";

// 函数声明
void initSensors();
bool readSensors(sensor_data_t *data);
void outputPWM(const control_output_t *output);
void updateAttitude(const sensor_data_t *data, drone_state_t *state, float dt);
void updateAltitude(const sensor_data_t *data, drone_state_t *state, float dt);
float pidUpdate(pid_controller_t *pid, float setpoint, float measurement, float dt);
void quaternionToEuler(const quaternion_t *q, euler_angles_t *euler);
void quaternionMultiply(const quaternion_t *q1, const quaternion_t *q2, quaternion_t *result);
void quaternionNormalize(quaternion_t *q);
void controlMixer(const euler_angles_t *attitude_error, float alt_error, float vel_z_error, control_output_t *output);
float applyDeadband(float value, float deadband);
float constrainFloat(float value, float min, float max);
void kalmanPredict(kalman_filter_t *kf, float dt);
void kalmanUpdate(kalman_filter_t *kf, float measurement);
bool checkSafety(const sensor_data_t *data, const drone_state_t *state, const control_output_t *output);
bool initWiFi();
bool sendUDPData(const udp_data_packet_t *packet);
void prepareUDPPacket(udp_data_packet_t *packet, const drone_state_t *state, 
                     const control_output_t *output, const pid_controller_t *pids);
void calculateAngularVelocity(sensor_data_t *data, float current_angles[3], uint32_t current_time);
void printStateAndPwmData(drone_state_t *state, control_output_t *output, sensor_data_t *data);
void printLastOperateStatus(BMP::eStatus_t eStatus);

// 初始化函数
void setup() {
    // 初始化串口通信
    Serial.begin(115200);

    // 初始化UDP
    if (udpStartSend == true) {
      // 初始化WiFi连接
      if (!initWiFi()) {
          Serial.println("WiFi连接失败！");
          // 继续执行，但UDP通信将不可用
      }
      udp.begin(udpPort);
    }

    // 初始化传感器
    initSensors();
    
    // 初始化无人机状态
    drone_state.attitude.w = 1.0;
    drone_state.attitude.x = 0.0;
    drone_state.attitude.y = 0.0;
    drone_state.attitude.z = 0.0;
    drone_state.altitude = 0.0;
    drone_state.timestamp = millis();
    
    // 初始化无人机配置
    drone_config.mass = 1.0;            // 1kg
    drone_config.motor_min_pwm = 5.0;   // 5%最小PWM
    drone_config.motor_max_pwm = 95.0;  // 95%最大PWM
    drone_config.hover_throttle = 50.0; // 50%悬停油门
    drone_config.max_angle.roll = 0.5;  // 约28.6度
    drone_config.max_angle.pitch = 0.5; // 约28.6度
    drone_config.max_angle.yaw = 1.0;   // 约57.3度
    drone_config.max_altitude = 50.0;   // 50米最大高度
    
    // 初始化控制输出
    control_output.motor1 = 0;
    control_output.motor2 = 0;
    control_output.motor3 = 0;
    control_output.motor4 = 0;
    control_output.armed = false;
    
    // 初始化PID控制器
    roll_pid.integral = 0;
    roll_pid.prev_error = 0;
    roll_pid.prev_measurement = 0;
    
    pitch_pid.integral = 0;
    pitch_pid.prev_error = 0;
    pitch_pid.prev_measurement = 0;
    
    yaw_pid.integral = 0;
    yaw_pid.prev_error = 0;
    yaw_pid.prev_measurement = 0;
    
    alt_pid.integral = 0;
    alt_pid.prev_error = 0;
    alt_pid.prev_measurement = 0;
    
    vel_z_pid.integral = 0;
    vel_z_pid.prev_error = 0;
    vel_z_pid.prev_measurement = 0;

    Serial.println("初始化完成");
}

// 主循环函数
void loop() {
    static uint32_t prev_time = 0;
    uint32_t current_time = millis();
    float dt = (current_time - prev_time) / 1000.0f;  // 计算时间差(秒)
    prev_time = current_time;
    
    if (dt <= 0 || dt > 0.1) {
        dt = 0.01;  // 确保时间差在合理范围内
    }
    
    // 读取传感器数据
    if (!readSensors(&sensor_data)) {
        // 传感器读取失败，进入安全模式
        control_output.armed = false;
        control_output.motor1 = 0;
        control_output.motor2 = 0;
        control_output.motor3 = 0;
        control_output.motor4 = 0;
        outputPWM(&control_output);
        return;
    }
    
    // 更新姿态估计
    updateAttitude(&sensor_data, &drone_state, dt);
    
    // 更新高度估计
    updateAltitude(&sensor_data, &drone_state, dt);
    
    // 限制目标姿态在安全范围内
    target_attitude.roll = constrainFloat(target_attitude.roll, 
                                         -drone_config.max_angle.roll, 
                                         drone_config.max_angle.roll);
    target_attitude.pitch = constrainFloat(target_attitude.pitch, 
                                          -drone_config.max_angle.pitch, 
                                          drone_config.max_angle.pitch);
    target_attitude.yaw = constrainFloat(target_attitude.yaw, 
                                        -drone_config.max_angle.yaw, 
                                        drone_config.max_angle.yaw);
    
    // 限制目标高度在安全范围内
    target_altitude = constrainFloat(target_altitude, 0, drone_config.max_altitude);
    
    // 计算姿态误差
    euler_angles_t attitude_error = {
        target_attitude.roll - drone_state.euler.roll,
        target_attitude.pitch - drone_state.euler.pitch,
        target_attitude.yaw - drone_state.euler.yaw
    };
    
    // 计算高度误差
    float alt_error = target_altitude - drone_state.altitude;
    
    // 计算垂直速度误差
    float vel_z_error = target_velocity_z - drone_state.velocity[2];
    
    // 更新PID控制器 - 使用测量值而不是误差的微分
    float roll_output = pidUpdate(&roll_pid, target_attitude.roll, drone_state.euler.roll, dt);
    float pitch_output = pidUpdate(&pitch_pid, target_attitude.pitch, drone_state.euler.pitch, dt);
    float yaw_output = pidUpdate(&yaw_pid, target_attitude.yaw, drone_state.euler.yaw, dt);
    float alt_output = pidUpdate(&alt_pid, target_altitude, drone_state.altitude, dt);
    float vel_z_output = pidUpdate(&vel_z_pid, target_velocity_z, drone_state.velocity[2], dt);
    
    // 垂直速度控制作为高度控制的辅助
    float combined_alt_output = alt_output + vel_z_output;
    
    // 控制混合器
    euler_angles_t pid_output = {roll_output, pitch_output, yaw_output};
    controlMixer(&pid_output, combined_alt_output, vel_z_error, &control_output);

    Serial.println("roll_pid");
    Serial.println(roll_output);
    
    // 安全检查
    if (!checkSafety(&sensor_data, &drone_state, &control_output)) {
        control_output.armed = false;
        control_output.motor1 = 0;
        control_output.motor2 = 0;
        control_output.motor3 = 0;
        control_output.motor4 = 0;
    }
    
    // 输出PWM信号
    outputPWM(&control_output);

    // 打印数据
    printStateAndPwmData(&drone_state, &control_output, &sensor_data);

    // 发送数据
    if (udpStartSend == true) {
      // 控制周期计数
      controlCycleCount++;

      // 每10个控制周期发送一次UDP数据
      if (controlCycleCount % udpSendInterval == 0) {
          udp_data_packet_t udp_packet;

          // 准备UDP数据包
          pid_controller_t pids[] = {roll_pid, pitch_pid, yaw_pid, alt_pid, vel_z_pid};
          prepareUDPPacket(&udp_packet, &drone_state, &control_output, pids);

          // 发送UDP数据包
          if (!sendUDPData(&udp_packet)) {
              Serial.println("UDP发送失败");
         }
      }
    }
}

// 初始化WiFi连接
bool initWiFi() {
    Serial.print("正在连接到WiFi: ");
    Serial.println(ssid);
    
    WiFi.begin(ssid, password);
    
    // 等待连接，最多等待10秒
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("");
        Serial.println("WiFi连接成功");
        Serial.print("IP地址: ");
        Serial.println(WiFi.localIP());
        return true;
    } else {
        Serial.println("");
        Serial.println("WiFi连接失败");
        return false;
    }
}

// 发送UDP数据
bool sendUDPData(const udp_data_packet_t *packet) {
    // 开始UDP数据包
    udp.beginPacket(udpAddress, udpPort);
    
    // 发送数据包
    size_t bytesSent = udp.write((const uint8_t*)packet, sizeof(udp_data_packet_t));
    
    // 结束数据包
    bool result = udp.endPacket() > 0;
    
    if (result && bytesSent == sizeof(udp_data_packet_t)) {
        Serial.printf("UDP数据包发送成功，序列号: %u\n", packet->sequence);
        return true;
    } else {
        Serial.printf("UDP数据包发送失败，已发送字节: %d/%d\n", 
                     bytesSent, sizeof(udp_data_packet_t));
        return false;
    }
}

// 准备UDP数据包
void prepareUDPPacket(udp_data_packet_t *packet, const drone_state_t *state, 
                     const control_output_t *output, const pid_controller_t *pids) {
    // 设置序列号和时间戳
    packet->sequence = udpSequence++;
    packet->timestamp = millis();
    
    // 设置姿态数据
    packet->attitude = state->euler;
    
    // 设置高度数据
    packet->altitude = state->altitude;
    
    // 设置速度数据
    packet->velocity[0] = state->velocity[0];
    packet->velocity[1] = state->velocity[1];
    packet->velocity[2] = state->velocity[2];
    
    // 设置电机输出数据
    packet->motor_output[0] = output->motor1;
    packet->motor_output[1] = output->motor2;
    packet->motor_output[2] = output->motor3;
    packet->motor_output[3] = output->motor4;
    
    // 设置PID输出数据
    packet->pid_output[0] = pids[0].output;  // roll
    packet->pid_output[1] = pids[1].output;  // pitch
    packet->pid_output[2] = pids[2].output;  // yaw
    packet->pid_output[3] = pids[3].output;  // alt
    packet->pid_output[4] = pids[4].output;  // vel_z
    
    // 设置状态标志位
    packet->status = 0;
    if (output->armed) packet->status |= 0x01;  // 第0位表示电机使能状态
    // 可以添加更多状态标志位
}

// 更新姿态估计（使用互补滤波融合陀螺仪和加速度计数据）
void updateAttitude(const sensor_data_t *data, drone_state_t *state, float dt) {
    // 四元数姿态更新算法
    quaternion_t dq;
    quaternion_t q_gyro = {0, data->gyro[0], data->gyro[1], data->gyro[2]};
    
    // 计算四元数导数: dq = 0.5 * q * ω
    quaternionMultiply(&state->attitude, &q_gyro, &dq);
    dq.w *= 0.5f;
    dq.x *= 0.5f;
    dq.y *= 0.5f;
    dq.z *= 0.5f;
    
    // 更新四元数: q = q + dq * dt
    state->attitude.w += dq.w * dt;
    state->attitude.x += dq.x * dt;
    state->attitude.y += dq.y * dt;
    state->attitude.z += dq.z * dt;
    
    // 归一化四元数
    quaternionNormalize(&state->attitude);
    
    // 转换为欧拉角
    quaternionToEuler(&state->attitude, &state->euler);
    
    // 使用加速度计校正俯仰和横滚（简化互补滤波）
    // 实际应用中应使用更复杂的算法如Mahony或Madgwick滤波器
    float accel_roll = atan2f(data->accel[1], data->accel[2]);
    float accel_pitch = atan2f(-data->accel[0], sqrtf(data->accel[1]*data->accel[1] + data->accel[2]*data->accel[2]));
    
    // 应用互补滤波
    float alpha = 0.98; // 陀螺仪数据权重
    state->euler.roll = alpha * state->euler.roll + (1 - alpha) * accel_roll;
    state->euler.pitch = alpha * state->euler.pitch + (1 - alpha) * accel_pitch;
    
    // 更新时间戳
    state->timestamp = data->timestamp;
}

// 更新高度估计（使用卡尔曼滤波融合气压计和加速度计数据）
void updateAltitude(const sensor_data_t *data, drone_state_t *state, float dt) {
    // 使用加速度计Z轴数据估计垂直加速度
    float vertical_acceleration = data->accel[2] - 9.8f; // 减去重力加速度
    
    // 预测垂直速度
    kalmanPredict(&vel_z_kalman, dt);
    vel_z_kalman.x += vertical_acceleration * dt; // 简化的速度预测
    
    // 更新垂直速度（使用加速度计数据）
    kalmanUpdate(&vel_z_kalman, vertical_acceleration);
    state->velocity[2] = vel_z_kalman.x;
    
    // 预测高度
    kalmanPredict(&alt_kalman, dt);
    alt_kalman.x += state->velocity[2] * dt; // 简化的高度预测
    
    // 更新高度（使用气压计数据）
    const float sea_level_pressure = 101325.0f; // 海平面标准大气压
    float measured_altitude = 44330.0f * (1.0f - powf(data->baro / sea_level_pressure, 0.1903f));
    
    // 温度补偿（简化）
    float temperature_compensation = 0.01f * (data->temp - 20.0f); // 假设20°C为参考温度
    measured_altitude += temperature_compensation;
    
    kalmanUpdate(&alt_kalman, measured_altitude);
    state->altitude = alt_kalman.x;
}

// PID控制器更新（使用测量值微分）
float pidUpdate(pid_controller_t *pid, float setpoint, float measurement, float dt) {
    // 计算误差
    float error = setpoint - measurement;
    
    // 应用死区
    error = applyDeadband(error, pid->deadband);
    
    // 比例项
    float proportional = pid->kp * error;
    
    // 积分项（带抗饱和）
    pid->integral += error * dt;
    pid->integral = constrainFloat(pid->integral, -pid->integral_limit, pid->integral_limit);
    float integral = pid->ki * pid->integral;
    
    // 微分项（使用测量值微分而不是误差微分）
    float derivative = -pid->kd * (measurement - pid->prev_measurement) / dt;
    
    // 更新上一次测量值
    pid->prev_measurement = measurement;
    pid->prev_error = error;
    
    // 计算总输出
    pid->output = proportional + integral + derivative;
    
    // 限制输出
    pid->output = constrainFloat(pid->output, -pid->output_limit, pid->output_limit);
    
    return pid->output;
}

// 四元数转欧拉角（Z-Y-X顺序）
void quaternionToEuler(const quaternion_t *q, euler_angles_t *euler) {
    // 横滚角 (x轴旋转)
    float sinr_cosp = 2.0f * (q->w * q->x + q->y * q->z);
    float cosr_cosp = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
    euler->roll = atan2f(sinr_cosp, cosr_cosp);
    
    // 俯仰角 (y轴旋转)
    float sinp = 2.0f * (q->w * q->y - q->z * q->x);
    if (fabsf(sinp) >= 1.0f) {
        euler->pitch = copysignf(M_PI / 2.0f, sinp); // 使用90度如果超出范围
    } else {
        euler->pitch = asinf(sinp);
    }
    
    // 偏航角 (z轴旋转)
    float siny_cosp = 2.0f * (q->w * q->z + q->x * q->y);
    float cosy_cosp = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
    euler->yaw = atan2f(siny_cosp, cosy_cosp);
}

// 四元数乘法
void quaternionMultiply(const quaternion_t *q1, const quaternion_t *q2, quaternion_t *result) {
    result->w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    result->x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    result->y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    result->z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
}

// 四元数归一化
void quaternionNormalize(quaternion_t *q) {
    float norm = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (norm > 0.0f) {
        q->w /= norm;
        q->x /= norm;
        q->y /= norm;
        q->z /= norm;
    }
}

// 控制混合器
void controlMixer(const euler_angles_t *attitude_error, float alt_error, float vel_z_error, control_output_t *output) {
    // 基础油门（高度控制 + 垂直速度控制）
    float base_throttle = drone_config.hover_throttle + alt_error + vel_z_error;
    
    // 限制基础油门在合理范围内
    base_throttle = constrainFloat(base_throttle, 
                                  drone_config.motor_min_pwm, 
                                  drone_config.motor_max_pwm);
    
    // 混合控制（X型四旋翼布局）
    // 电机1: 前左, 电机2: 前右, 电机3: 后右, 电机4: 后左
    output->motor1 = base_throttle - attitude_error->pitch + attitude_error->roll - attitude_error->yaw;
    output->motor2 = base_throttle - attitude_error->pitch - attitude_error->roll + attitude_error->yaw;
    output->motor3 = base_throttle + attitude_error->pitch - attitude_error->roll - attitude_error->yaw;
    output->motor4 = base_throttle + attitude_error->pitch + attitude_error->roll + attitude_error->yaw;
    
    // 限制PWM输出在允许范围内
    output->motor1 = constrainFloat(output->motor1, drone_config.motor_min_pwm, drone_config.motor_max_pwm);
    output->motor2 = constrainFloat(output->motor2, drone_config.motor_min_pwm, drone_config.motor_max_pwm);
    output->motor3 = constrainFloat(output->motor3, drone_config.motor_min_pwm, drone_config.motor_max_pwm);
    output->motor4 = constrainFloat(output->motor4, drone_config.motor_min_pwm, drone_config.motor_max_pwm);
}

// 应用死区
float applyDeadband(float value, float deadband) {
    if (fabsf(value) < deadband) {
        return 0.0f;
    }
    return value;
}

// 限制浮点数范围
float constrainFloat(float value, float min, float max) {
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

// 卡尔曼滤波预测步骤
void kalmanPredict(kalman_filter_t *kf, float dt) {
    // 简化的一维卡尔曼滤波预测
    kf->x = kf->x; // 状态保持不变（简化模型）
    kf->p = kf->p + kf->q * dt; // 增加估计不确定性
}

// 卡尔曼滤波更新步骤
void kalmanUpdate(kalman_filter_t *kf, float measurement) {
    // 简化的一维卡尔曼滤波更新
    kf->k = kf->p / (kf->p + kf->r); // 计算卡尔曼增益
    kf->x = kf->x + kf->k * (measurement - kf->x); // 更新状态估计
    kf->p = (1 - kf->k) * kf->p; // 更新估计不确定性
}

// 安全检查函数
bool checkSafety(const sensor_data_t *data, const drone_state_t *state, const control_output_t *output) {
    // 检查姿态是否超过安全限制
    if (fabsf(state->euler.roll) > drone_config.max_angle.roll ||
        fabsf(state->euler.pitch) > drone_config.max_angle.pitch) {
        return false;
    }
    
    // 检查高度是否超过安全限制
    if (state->altitude > drone_config.max_altitude) {
        return false;
    }
    
    // 检查传感器数据是否有效
    if (!data->valid) {
        return false;
    }
    
    // 检查电机输出是否在合理范围内
    if (output->motor1 < drone_config.motor_min_pwm || output->motor1 > drone_config.motor_max_pwm ||
        output->motor2 < drone_config.motor_min_pwm || output->motor2 > drone_config.motor_max_pwm ||
        output->motor3 < drone_config.motor_min_pwm || output->motor3 > drone_config.motor_max_pwm ||
        output->motor4 < drone_config.motor_min_pwm || output->motor4 > drone_config.motor_max_pwm) {
        return false;
    }
    
    return true;
}

// 从角度值计算角速度的函数
void calculateAngularVelocity(sensor_data_t *data, float current_angles[3], uint32_t current_time) {
    // 如果是第一次调用，初始化上一次的角度值并返回零角速度
    if (last_angle_time == 0) {
        last_angles[0] = current_angles[0];
        last_angles[1] = current_angles[1];
        last_angles[2] = current_angles[2];
        last_angle_time = current_time;
        
        // 设置角速度为零
        data->gyro[0] = 0;
        data->gyro[1] = 0;
        data->gyro[2] = 0;
        return;
    }
    
    // 计算时间间隔（秒）
    float dt = (current_time - last_angle_time) / 1000.0f;
    
    // 确保时间间隔为正且合理
    if (dt <= 0 || dt > 1.0f) {
        dt = 0.01f;  // 使用默认时间间隔
    }
    
    // 计算角速度（角度变化率）
    // 注意：需要处理角度环绕问题（例如从359度到1度的变化）
    for (int i = 0; i < 3; i++) {
        float angle_diff = current_angles[i] - last_angles[i];
        
        // 处理角度环绕（假设角度范围是-180到180度或0到360度）
        if (angle_diff > 180.0f) {
            angle_diff -= 360.0f;
        } else if (angle_diff < -180.0f) {
            angle_diff += 360.0f;
        }
        
        // 计算角速度（度/秒）
        float angular_velocity_deg = angle_diff / dt;
        
        // 转换为弧度/秒（飞控算法通常使用弧度）
        data->gyro[i] = angular_velocity_deg * (M_PI / 180.0f);
    }
    
    // 更新上一次的角度值和时间戳
    last_angles[0] = current_angles[0];
    last_angles[1] = current_angles[1];
    last_angles[2] = current_angles[2];
    last_angle_time = current_time;
}

void initSensors() {
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

    delay(100);

    while (!compass.begin())
    {
      Serial.println("Could not find a valid 5883 sensor, check wiring!");
      delay(500);
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

    Wire.begin();
  
    delay(5);
    //sixDOF.init(); //begin the IMU
    delay(5);
}

bool readSensors(sensor_data_t *data) {
    uint32_t current_time = millis();
    data->timestamp = millis();
    
    float   temp = bmp.getTemperature();
    uint32_t    press = bmp.getPressure();
    float   alti = bmp.calAltitude(SEA_LEVEL_PRESSURE, press);

    //Serial.println();
    //Serial.println("======== start print ========");
    //Serial.print("temperature (unit Celsius): "); Serial.println(temp);
    //Serial.print("pressure (unit pa):         "); Serial.println(press);
    //Serial.print("altitude (unit meter):      "); Serial.println(alti);
    //Serial.println("========  end print  ========");

    data->alti = alti;
    data->baro = press;
    data->temp = temp;

    float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
    compass.setDeclinationAngle(declinationAngle);
    sVector_t mag = compass.readRaw();
    compass.getHeadingDegrees();
    //Serial.print("X:");
    //Serial.print(mag.XAxis);
    //Serial.print(" Y:");
    //Serial.print(mag.YAxis);
    //Serial.print(" Z:");
    //Serial.println(mag.ZAxis);
    //Serial.print("Degress = ");
    //Serial.println(mag.HeadingDegress);

    //sixDOF.getEuler(angles);
    //Serial.print(angles[0]);
    //Serial.print(" | ");  
    //Serial.print(angles[1]);
    //Serial.print(" | ");
    //Serial.println(angles[2]);

    float mag_angles[3];
    mag_angles[0] = mag.XAxis;
    mag_angles[1] = mag.YAxis;
    mag_angles[2] = mag.ZAxis;

    calculateAngularVelocity(data, mag_angles, current_time);

    data->valid = true;
    return true;
}

void outputPWM(const control_output_t *output) {
    // 将PWM值输出到电调
    // 将百分比转换为实际的PWM脉冲宽度
}

// 打印无人机状态和电机的pwm值
void printStateAndPwmData(drone_state_t *state, control_output_t *output, sensor_data_t *data) {
    Serial.println("----------------------- sensor -------------------------");
    Serial.print("baro:             ");
    Serial.println(data->baro);
    Serial.print("temp:             ");
    Serial.println(data->temp);
    Serial.print("alti:             ");
    Serial.println(data->alti);
    Serial.print("timestamp:        ");
    Serial.println(data->timestamp);
    Serial.print("accel[3] xyz:     ");
    Serial.print(data->accel[0]);       Serial.print(" ");
    Serial.print(data->accel[1]);       Serial.print(" ");
    Serial.println(data->accel[2]);
    Serial.print("gyro[3] xyz:      ");
    Serial.print(data->gyro[0]);        Serial.print(" ");
    Serial.print(data->gyro[1]);        Serial.print(" ");
    Serial.println(data->gyro[2]);
    Serial.println("----------------------- state --------------------------");
    Serial.print("quaternion wxyz:  ");
    Serial.print(state->attitude->w);   Serial.print(" ");
    Serial.print(state->attitude->x);   Serial.print(" ");
    Serial.print(state->attitude->y);   Serial.print(" ");
    Serial.println(state->attitude->z);
    Serial.print("euler_angles rpy: ");
    Serial.print(state->euler->roll);   Serial.print(" ");
    Serial.print(state->euler->pitch);  Serial.print(" ");
    Serial.println(state->euler->yaw);
    Serial.print("altitude:         ");
    Serial.println(state->altitude);
    Serial.print("velocity[3] xyz:  ");
    Serial.print(state->velocity[0]);   Serial.print(" ");
    Serial.print(state->velocity[1]);   Serial.print(" ");
    Serial.println(state->velocity[2]);
    Serial.print("position[3] xyz:  ");
    Serial.print(state->position[0]);   Serial.print(" ");
    Serial.print(state->position[1]);   Serial.print(" ");
    Serial.println(state->position[2]);
    Serial.print("motor1:           ");
    Serial.println(output->motor1);
    Serial.print("motor2:           ");
    Serial.println(output->motor2);
    Serial.print("motor3:           ");
    Serial.println(output->motor3);
    Serial.print("motor4:           ");
    Serial.println(output->motor4);
    Serial.print("armed:            ");
    Serial.println(output->armed);
    Serial.println("------------------------ off ---------------------------");
}

// show last sensor operate status
void printLastOperateStatus(BMP::eStatus_t eStatus) {
  switch(eStatus) {
  case BMP::eStatusOK:    Serial.println("everything ok"); break;
  case BMP::eStatusErr:   Serial.println("unknow error"); break;
  case BMP::eStatusErrDeviceNotDetected:    Serial.println("device not detected"); break;
  case BMP::eStatusErrParameter:    Serial.println("parameter error"); break;
  default: Serial.println("unknow status"); break;
  }
}
