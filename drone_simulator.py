#!/usr/bin/env python3
"""
ESP32无人机飞控模拟器
模拟无人机动力学、传感器和控制算法
"""

import numpy as np
import math
import time
import json
from dataclasses import dataclass
from typing import List, Tuple, Optional
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ============================================================================
# 数据类型定义
# ============================================================================

@dataclass
class SensorData:
    """传感器数据"""
    accel: np.ndarray  # 加速度计数据 (m/s^2) [x, y, z]
    gyro: np.ndarray   # 陀螺仪数据 (rad/s) [x, y, z]
    mag: np.ndarray    # 磁力计数据 (μT) [x, y, z]
    baro: float        # 气压计数据 (Pa)
    temp: float        # 温度计数据 (°C)
    alti: float        # 高度数据 (m)
    timestamp: float   # 时间戳 (s)
    valid: bool        # 数据有效性标志

@dataclass
class Quaternion:
    """四元数"""
    w: float
    x: float
    y: float
    z: float
    
    def to_array(self) -> np.ndarray:
        return np.array([self.w, self.x, self.y, self.z])
    
    @classmethod
    def from_array(cls, arr):
        return cls(arr[0], arr[1], arr[2], arr[3])
    
    def normalize(self):
        norm = math.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)
        if norm > 0:
            self.w /= norm
            self.x /= norm
            self.y /= norm
            self.z /= norm
    
    def conjugate(self):
        return Quaternion(self.w, -self.x, -self.y, -self.z)
    
    def multiply(self, other):
        w = self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z
        x = self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y
        y = self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x
        z = self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w
        return Quaternion(w, x, y, z)

@dataclass
class EulerAngles:
    """欧拉角"""
    roll: float   # 横滚角 (rad)
    pitch: float  # 俯仰角 (rad)
    yaw: float    # 偏航角 (rad)

@dataclass
class DroneState:
    """无人机状态"""
    attitude: Quaternion    # 当前姿态(四元数)
    euler: EulerAngles     # 当前姿态(欧拉角)
    altitude: float        # 当前高度(m)
    velocity: np.ndarray   # 当前速度(m/s) [x, y, z]
    position: np.ndarray   # 当前位置(m) [x, y, z]
    timestamp: float       # 状态时间戳

@dataclass
class PIDController:
    """PID控制器"""
    kp: float           # 比例系数
    ki: float           # 积分系数
    kd: float           # 微分系数
    integral: float     # 积分项
    prev_error: float   # 上一次误差
    prev_measurement: float  # 上一次测量值
    output: float       # 输出值
    output_limit: float # 输出限制
    integral_limit: float  # 积分项限制
    deadband: float     # 死区
    
    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        """更新PID控制器"""
        # 计算误差
        error = setpoint - measurement
        
        # 应用死区
        if abs(error) < self.deadband:
            error = 0.0
        
        # 比例项
        proportional = self.kp * error
        
        # 积分项（带抗饱和）
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)
        integral = self.ki * self.integral
        
        # 微分项（使用测量值微分）
        derivative = -self.kd * (measurement - self.prev_measurement) / dt if dt > 0 else 0
        
        # 更新上一次测量值
        self.prev_measurement = measurement
        self.prev_error = error
        
        # 计算总输出
        self.output = proportional + integral + derivative
        
        # 限制输出
        self.output = np.clip(self.output, -self.output_limit, self.output_limit)
        
        return self.output

@dataclass
class KalmanFilter:
    """卡尔曼滤波器（简化一维）"""
    x: float        # 状态估计值
    p: float        # 估计误差协方差
    q: float        # 过程噪声协方差
    r: float        # 测量噪声协方差
    k: float        # 卡尔曼增益
    
    def predict(self, dt: float):
        """预测步骤"""
        # 状态保持不变（简化模型）
        self.x = self.x
        # 增加估计不确定性
        self.p = self.p + self.q * dt
    
    def update(self, measurement: float):
        """更新步骤"""
        # 计算卡尔曼增益
        self.k = self.p / (self.p + self.r)
        # 更新状态估计
        self.x = self.x + self.k * (measurement - self.x)
        # 更新估计不确定性
        self.p = (1 - self.k) * self.p

@dataclass
class ControlOutput:
    """控制输出"""
    motor1: float   # 电机1 PWM值 (0-100%)
    motor2: float   # 电机2 PWM值 (0-100%)
    motor3: float   # 电机3 PWM值 (0-100%)
    motor4: float   # 电机4 PWM值 (0-100%)
    armed: bool     # 电机使能标志

@dataclass
class DroneConfig:
    """无人机配置"""
    mass: float             # 无人机质量 (kg)
    motor_min_pwm: float    # 电机最小PWM值 (%)
    motor_max_pwm: float    # 电机最大PWM值 (%)
    hover_throttle: float   # 悬停油门值 (%)
    max_angle: EulerAngles  # 最大允许角度 (rad)
    max_altitude: float     # 最大允许高度 (m)
    motor_constant: float   # 电机推力常数 (N/%)
    drag_coefficient: float # 阻力系数

# ============================================================================
# 无人机模拟器类
# ============================================================================

class DroneSimulator:
    """无人机模拟器"""
    
    def __init__(self, config: Optional[DroneConfig] = None):
        # 默认配置
        if config is None:
            config = DroneConfig(
                mass=1.0,
                motor_min_pwm=5.0,
                motor_max_pwm=95.0,
                hover_throttle=50.0,
                max_angle=EulerAngles(0.5, 0.5, 1.0),  # 约28.6°, 28.6°, 57.3°
                max_altitude=50.0,
                motor_constant=0.02,  # 20g/% 推力
                drag_coefficient=0.1
            )
        self.config = config
        
        # 初始化状态
        self.state = DroneState(
            attitude=Quaternion(1.0, 0.0, 0.0, 0.0),
            euler=EulerAngles(0.0, 0.0, 0.0),
            altitude=0.0,
            velocity=np.zeros(3),
            position=np.zeros(3),
            timestamp=0.0
        )
        
        # 初始化控制输出
        self.control_output = ControlOutput(0.0, 0.0, 0.0, 0.0, False)
        
        # 初始化PID控制器
        self.roll_pid = PIDController(3.0, 0.8, 0.2, 0, 0, 0, 0, 45.0, 25.0, 0.05)
        self.pitch_pid = PIDController(3.0, 0.8, 0.2, 0, 0, 0, 0, 45.0, 25.0, 0.05)
        self.yaw_pid = PIDController(2.0, 0.5, 0.1, 0, 0, 0, 0, 45.0, 25.0, 0.1)
        self.alt_pid = PIDController(1.5, 0.4, 0.5, 0, 0, 0, 0, 100.0, 50.0, 0.2)
        self.vel_z_pid = PIDController(1.2, 0.3, 0.2, 0, 0, 0, 0, 50.0, 25.0, 0.1)
        
        # 初始化卡尔曼滤波器
        self.alt_kalman = KalmanFilter(0, 1, 0.01, 0.1, 0)
        self.vel_z_kalman = KalmanFilter(0, 1, 0.01, 0.2, 0)
        
        # 目标状态
        self.target_attitude = EulerAngles(0.0, 0.0, 0.0)
        self.target_altitude = 5.0  # 目标高度5米
        self.target_velocity_z = 0.0
        
        # 传感器噪声参数
        self.accel_noise = 0.05  # 加速度计噪声 (m/s^2)
        self.gyro_noise = 0.01   # 陀螺仪噪声 (rad/s)
        self.mag_noise = 0.1     # 磁力计噪声 (μT)
        self.baro_noise = 10.0   # 气压计噪声 (Pa)
        
        # 模拟时间
        self.sim_time = 0.0
        self.dt = 0.01  # 时间步长 (10ms)
        
        # 历史记录
        self.history = {
            'time': [],
            'altitude': [],
            'roll': [],
            'pitch': [],
            'yaw': [],
            'motor_outputs': [],
            'target_altitude': []
        }
    
    def simulate_sensors(self) -> SensorData:
        """模拟传感器数据（带噪声）"""
        # 真实物理量（无噪声）
        g = 9.81  # 重力加速度
        
        # 计算真实加速度（在机体坐标系中）
        # 简化模型：假设只有重力和电机推力
        total_thrust = (self.control_output.motor1 + self.control_output.motor2 + 
                       self.control_output.motor3 + self.control_output.motor4) / 4.0
        thrust_accel = total_thrust * self.config.motor_constant / self.config.mass
        
        # 将重力转换到机体坐标系
        q = self.state.attitude
        gravity_body = self.rotate_vector_by_quaternion(np.array([0, 0, -g]), q)
        
        # 机体加速度 = 推力 + 重力 + 阻力
        drag_accel = -self.config.drag_coefficient * self.state.velocity
        true_accel = np.array([0, 0, thrust_accel]) + gravity_body + drag_accel
        
        # 添加噪声
        accel = true_accel + np.random.normal(0, self.accel_noise, 3)
        
        # 陀螺仪数据（角速度）
        # 简化：假设角速度与姿态误差成正比
        true_gyro = np.array([
            self.target_attitude.roll - self.state.euler.roll,
            self.target_attitude.pitch - self.state.euler.pitch,
            self.target_attitude.yaw - self.state.euler.yaw
        ]) * 2.0
        
        gyro = true_gyro + np.random.normal(0, self.gyro_noise, 3)
        
        # 磁力计数据（简化模型）
        # 假设地磁场在NED坐标系中为[0, 1, 0] μT
        earth_mag = np.array([0, 1, 0])
        mag_body = self.rotate_vector_by_quaternion(earth_mag, q)
        mag = mag_body + np.random.normal(0, self.mag_noise, 3)
        
        # 气压计数据（根据高度计算）
        sea_level_pressure = 101325.0  # 海平面标准大气压
        # 气压随高度变化公式: P = P0 * exp(-h/H)
        scale_height = 8500.0  # 大气标高 (m)
        true_pressure = sea_level_pressure * math.exp(-self.state.altitude / scale_height)
        baro = true_pressure + np.random.normal(0, self.baro_noise)
        
        # 温度数据（简化）
        temp = 20.0 + self.state.altitude * -0.0065  # 温度递减率
        
        # 高度数据（从气压计算）
        alti = 44330.0 * (1.0 - math.pow(baro / sea_level_pressure, 0.1903))
        
        return SensorData(
            accel=accel,
            gyro=gyro,
            mag=mag,
            baro=baro,
            temp=temp,
            alti=alti,
            timestamp=self.sim_time,
            valid=True
        )
    
    def rotate_vector_by_quaternion(self, v: np.ndarray, q: Quaternion) -> np.ndarray:
        """使用四元数旋转向量"""
        # 将向量转换为纯四元数
        v_q = Quaternion(0, v[0], v[1], v[2])
        
        # 旋转: v' = q * v * q^-1
        q_conj = q.conjugate()
        result = q.multiply(v_q).multiply(q_conj)
        
        return np.array([result.x, result.y, result.z])
    
    def quaternion_to_euler(self, q: Quaternion) -> EulerAngles:
        """四元数转欧拉角（Z-Y-X顺序）"""
        # 横滚角 (x轴旋转)
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # 俯仰角 (y轴旋转)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)
        
        # 偏航角 (z轴旋转)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return EulerAngles(roll, pitch, yaw)
    
    def update_attitude(self, sensor_data: SensorData):
        """更新姿态估计（使用互补滤波）"""
        # 四元数姿态更新算法
        q = self.state.attitude
        
        # 陀螺仪四元数
        q_gyro = Quaternion(0, sensor_data.gyro[0], sensor_data.gyro[1], sensor_data.gyro[2])
        
        # 计算四元数导数: dq = 0.5 * q * ω
        dq = q.multiply(q_gyro)
        dq.w *= 0.5
        dq.x *= 0.5
        dq.y *= 0.5
        dq.z *= 0.5
        
        # 更新四元数: q = q + dq * dt
        q.w += dq.w * self.dt
        q.x += dq.x * self.dt
        q.y += dq.y * self.dt
        q.z += dq.z * self.dt
        
        # 归一化四元数
        q.normalize()
        
        # 转换为欧拉角
        self.state.euler = self.quaternion_to_euler(q)
        
        # 使用加速度计校正俯仰和横滚（互补滤波）
        accel = sensor_data.accel
        if np.linalg.norm(accel) > 0.1:  # 避免除零
            accel_normalized = accel / np.linalg.norm(accel)
            accel_roll = math.atan2(accel_normalized[1], accel_normalized[2])
            accel_pitch = math.atan2(-accel_normalized[0], 
                                    math.sqrt(accel_normalized[1]**2 + accel_normalized[2]**2))
            
            # 应用互补滤波
            alpha = 0.98  # 陀螺仪数据权重
            self.state.euler.roll = alpha * self.state.euler.roll + (1 - alpha) * accel_roll
            self.state.euler.pitch = alpha * self.state.euler.pitch + (1 - alpha) * accel_pitch
        
        # 更新时间戳
        self.state.timestamp = sensor_data.timestamp
    
    def update_altitude(self, sensor_data: SensorData):
        """更新高度估计（使用卡尔曼滤波）"""
        # 使用加速度计Z轴数据估计垂直加速度
        vertical_acceleration = sensor_data.accel[2] - 9.81  # 减去重力加速度
        
        # 预测垂直速度
        self.vel_z_kalman.predict(self.dt)
        self.vel_z_kalman.x += vertical_acceleration * self.dt  # 简化的速度预测
        
        # 更新垂直速度（使用加速度计数据）
        self.vel_z_kalman.update(vertical_acceleration)
        self.state.velocity[2] = self.vel_z_kalman.x
        
        # 预测高度
        self.alt_kalman.predict(self.dt)
        self.alt_kalman.x += self.state.velocity[2] * self.dt  # 简化的高度预测
        
        # 更新高度（使用气压计数据）
        measured_altitude = sensor_data.alti
        
        # 温度补偿（简化）
        temperature_compensation = 0.01 * (sensor_data.temp - 20.0)  # 假设20°C为参考温度
        measured_altitude += temperature_compensation
        
        self.alt_kalman.update(measured_altitude)
        self.state.altitude = self.alt_kalman.x
    
    def control_mixer(self, attitude_error: EulerAngles, alt_error: float, vel_z_error: float):
        """控制混合器"""
        # 基础油门（高度控制 + 垂直速度控制）
        base_throttle = self.config.hover_throttle + alt_error + vel_z_error
        
        # 限制基础油门在合理范围内
        base_throttle = np.clip(base_throttle, 
                               self.config.motor_min_pwm, 
                               self.config.motor_max_pwm)
        
        # 混合控制（X型四旋翼布局）
        # 电机1: 前左, 电机2: 前右, 电机3: 后右, 电机4: 后左
        self.control_output.motor1 = base_throttle - attitude_error.pitch + attitude_error.roll - attitude_error.yaw
        self.control_output.motor2 = base_throttle - attitude_error.pitch - attitude_error.roll + attitude_error.yaw
        self.control_output.motor3 = base_throttle + attitude_error.pitch - attitude_error.roll - attitude_error.yaw
        self.control_output.motor4 = base_throttle + attitude_error.pitch + attitude_error.roll + attitude_error.yaw
        
        # 限制PWM输出在允许范围内
        self.control_output.motor1 = np.clip(self.control_output.motor1, 
                                            self.config.motor_min_pwm, 
                                            self.config.motor_max_pwm)
        self.control_output.motor2 = np.clip(self.control_output.motor2, 
                                            self.config.motor_min_pwm, 
                                            self.config.motor_max_pwm)
        self.control_output.motor3 = np.clip(self.control_output.motor3, 
                                            self.config.motor_min_pwm, 
                                            self.config.motor_max_pwm)
        self.control_output.motor4 = np.clip(self.control_output.motor4, 
                                            self.config.motor_min_pwm, 
                                            self.config.motor_max_pwm)
    
    def check_safety(self, sensor_data: SensorData) -> bool:
        """安全检查函数"""
        # 检查姿态是否超过安全限制
        if (abs(self.state.euler.roll) > self.config.max_angle.roll or
            abs(self.state.euler.pitch) > self.config.max_angle.pitch):
            return False
        
        # 检查高度是否超过安全限制
        if self.state.altitude > self.config.max_altitude:
            return False
        
        # 检查传感器数据是否有效
        if not sensor_data.valid:
            return False
        
        # 检查电机输出是否在合理范围内
        motors = [self.control_output.motor1, self.control_output.motor2,
                 self.control_output.motor3, self.control_output.motor4]
        
        for motor in motors:
            if (motor < self.config.motor_min_pwm or 
                motor > self.config.motor_max_pwm):
                return False
        
        return True
    
    def update_physics(self):
        """更新物理模型"""
        # 计算总推力
        total_thrust = (self.control_output.motor1 + self.control_output.motor2 + 
                       self.control_output.motor3 + self.control_output.motor4) / 4.0
        thrust_force = total_thrust * self.config.motor_constant
        
        # 将推力转换到世界坐标系
        q = self.state.attitude
        thrust_world = self.rotate_vector_by_quaternion(np.array([0, 0, thrust_force]), q)
        
        # 计算合力（推力 + 重力 + 阻力）
        gravity = np.array([0, 0, -9.81 * self.config.mass])
        drag_force = -self.config.drag_coefficient * self.state.velocity
        
        total_force = thrust_world + gravity + drag_force
        
        # 计算加速度
        acceleration = total_force / self.config.mass
        
        # 更新速度（欧拉积分）
        self.state.velocity += acceleration * self.dt
        
        # 更新位置
        self.state.position += self.state.velocity * self.dt
        
        # 更新高度（从位置Z坐标获取）
        self.state.altitude = max(0, self.state.position[2])
    
    def run_control_cycle(self):
        """运行一个控制周期"""
        # 模拟传感器数据
        sensor_data = self.simulate_sensors()
        
        # 更新姿态估计
        self.update_attitude(sensor_data)
        
        # 更新高度估计
        self.update_altitude(sensor_data)
        
        # 限制目标姿态在安全范围内
        self.target_attitude.roll = np.clip(self.target_attitude.roll,
                                           -self.config.max_angle.roll,
                                           self.config.max_angle.roll)
        self.target_attitude.pitch = np.clip(self.target_attitude.pitch,
                                            -self.config.max_angle.pitch,
                                            self.config.max_angle.pitch)
        self.target_attitude.yaw = np.clip(self.target_attitude.yaw,
                                          -self.config.max_angle.yaw,
                                          self.config.max_angle.yaw)
        
        # 限制目标高度在安全范围内
        self.target_altitude = np.clip(self.target_altitude, 0, self.config.max_altitude)
        
        # 计算姿态误差
        attitude_error = EulerAngles(
            self.target_attitude.roll - self.state.euler.roll,
            self.target_attitude.pitch - self.state.euler.pitch,
            self.target_attitude.yaw - self.state.euler.yaw
        )
        
        # 计算高度误差
        alt_error = self.target_altitude - self.state.altitude
        
        # 计算垂直速度误差
        vel_z_error = self.target_velocity_z - self.state.velocity[2]
        
        # 更新PID控制器
        roll_output = self.roll_pid.update(self.target_attitude.roll, self.state.euler.roll, self.dt)
        pitch_output = self.pitch_pid.update(self.target_attitude.pitch, self.state.euler.pitch, self.dt)
        yaw_output = self.yaw_pid.update(self.target_attitude.yaw, self.state.euler.yaw, self.dt)
        alt_output = self.alt_pid.update(self.target_altitude, self.state.altitude, self.dt)
        vel_z_output = self.vel_z_pid.update(self.target_velocity_z, self.state.velocity[2], self.dt)
        
        # 垂直速度控制作为高度控制的辅助
        combined_alt_output = alt_output + vel_z_output
        
        # 控制混合器
        pid_output = EulerAngles(roll_output, pitch_output, yaw_output)
        self.control_mixer(pid_output, combined_alt_output, vel_z_error)
        
        # 安全检查
        if not self.check_safety(sensor_data):
            self.control_output.armed = False
            self.control_output.motor1 = 0
            self.control_output.motor2 = 0
            self.control_output.motor3 = 0
            self.control_output.motor4 = 0
        else:
            self.control_output.armed = True
        
        # 更新物理模型
        self.update_physics()
        
        # 记录历史数据
        self.history['time'].append(self.sim_time)
        self.history['altitude'].append(self.state.altitude)
        self.history['roll'].append(math.degrees(self.state.euler.roll))
        self.history['pitch'].append(math.degrees(self.state.euler.pitch))
        self.history['yaw'].append(math.degrees(self.state.euler.yaw))
        self.history['motor_outputs'].append([
            self.control_output.motor1,
            self.control_output.motor2,
            self.control_output.motor3,
            self.control_output.motor4
        ])
        self.history['target_altitude'].append(self.target_altitude)
        
        # 更新时间
        self.sim_time += self.dt
    
    def run_simulation(self, duration: float = 30.0):
        """运行模拟"""
        num_steps = int(duration / self.dt)
        
        print(f"开始模拟，持续时间: {duration}秒，步数: {num_steps}")
        print("=" * 60)
        
        # 初始状态
        print(f"初始状态: 高度={self.state.altitude:.2f}m, "
              f"横滚={math.degrees(self.state.euler.roll):.1f}°, "
              f"俯仰={math.degrees(self.state.euler.pitch):.1f}°")
        
        # 模拟控制循环
        for step in range(num_steps):
            # 每5秒改变一次目标高度
            if step % 500 == 0 and step > 0:
                # 在2米和8米之间切换目标高度
                if self.target_altitude < 5.0:
                    self.target_altitude = 8.0
                else:
                    self.target_altitude = 2.0
                print(f"时间 {self.sim_time:.1f}s: 目标高度改为 {self.target_altitude:.1f}m")
            
            # 每10秒添加一个姿态扰动
            if step % 1000 == 0 and step > 0:
                self.target_attitude.roll = np.random.uniform(-0.2, 0.2)  # ±11.5°
                self.target_attitude.pitch = np.random.uniform(-0.2, 0.2)  # ±11.5°
                print(f"时间 {self.sim_time:.1f}s: 姿态扰动 - "
                      f"横滚={math.degrees(self.target_attitude.roll):.1f}°, "
                      f"俯仰={math.degrees(self.target_attitude.pitch):.1f}°")
            
            self.run_control_cycle()
            
            # 每100步打印一次状态
            if step % 100 == 0:
                print(f"时间 {self.sim_time:.2f}s: "
                      f"高度={self.state.altitude:.2f}m, "
                      f"目标={self.target_altitude:.1f}m, "
                      f"电机=[{self.control_output.motor1:.1f}%, "
                      f"{self.control_output.motor2:.1f}%, "
                      f"{self.control_output.motor3:.1f}%, "
                      f"{self.control_output.motor4:.1f}%]")
        
        print("=" * 60)
        print("模拟完成")
        
        return self.history
    
    def plot_results(self, history: dict):
        """绘制模拟结果"""
        fig, axes = plt.subplots(3, 2, figsize=(12, 10))
        
        # 高度跟踪
        ax = axes[0, 0]
        ax.plot(history['time'], history['altitude'], 'b-', label='实际高度')
        ax.plot(history['time'], history['target_altitude'], 'r--', label='目标高度')
        ax.set_xlabel('时间 (s)')
        ax.set_ylabel('高度 (m)')
        ax.set_title('高度跟踪')
        ax.legend()
        ax.grid(True)
        
        # 姿态角
        ax = axes[0, 1]
        ax.plot(history['time'], history['roll'], 'r-', label='横滚')
        ax.plot(history['time'], history['pitch'], 'g-', label='俯仰')
        ax.plot(history['time'], history['yaw'], 'b-', label='偏航')
        ax.set_xlabel('时间 (s)')
        ax.set_ylabel('角度 (°)')
        ax.set_title('姿态角')
        ax.legend()
        ax.grid(True)
        
        # 电机输出
        ax = axes[1, 0]
        motor_outputs = np.array(history['motor_outputs'])
        ax.plot(history['time'], motor_outputs[:, 0], 'r-', label='电机1')
        ax.plot(history['time'], motor_outputs[:, 1], 'g-', label='电机2')
        ax.plot(history['time'], motor_outputs[:, 2], 'b-', label='电机3')
        ax.plot(history['time'], motor_outputs[:, 3], 'y-', label='电机4')
        ax.set_xlabel('时间 (s)')
        ax.set_ylabel('PWM (%)')
        ax.set_title('电机输出')
        ax.legend()
        ax.grid(True)
        
        # 高度误差
        ax = axes[1, 1]
        altitude_error = np.array(history['target_altitude']) - np.array(history['altitude'])
        ax.plot(history['time'], altitude_error, 'k-')
        ax.set_xlabel('时间 (s)')
        ax.set_ylabel('高度误差 (m)')
        ax.set_title('高度跟踪误差')
        ax.grid(True)
        
        # 姿态角统计
        ax = axes[2, 0]
        angles = [history['roll'], history['pitch'], history['yaw']]
        labels = ['横滚', '俯仰', '偏航']
        colors = ['red', 'green', 'blue']
        
        for i, (angle, label, color) in enumerate(zip(angles, labels, colors)):
            ax.hist(angle, bins=30, alpha=0.5, label=label, color=color)
        
        ax.set_xlabel('角度 (°)')
        ax.set_ylabel('频率')
        ax.set_title('姿态角分布')
        ax.legend()
        ax.grid(True)
        
        # 电机输出统计
        ax = axes[2, 1]
        for i in range(4):
            ax.hist(motor_outputs[:, i], bins=30, alpha=0.25, label=f'电机{i+1}')
        
        ax.set_xlabel('PWM (%)')
        ax.set_ylabel('频率')
        ax.set_title('电机输出分布')
        ax.legend()
        ax.grid(True)
        
        plt.tight_layout()
        plt.show()
    
    def save_results(self, history: dict, filename: str = "simulation_results.json"):
        """保存模拟结果到JSON文件"""
        # 转换numpy数组为Python列表
        serializable_history = {}
        for key, value in history.items():
            if isinstance(value, list) and len(value) > 0 and isinstance(value[0], list):
                # 嵌套列表（如motor_outputs）
                serializable_history[key] = [list(v) for v in value]
            else:
                serializable_history[key] = value
        
        with open(filename, 'w') as f:
            json.dump(serializable_history, f, indent=2)
        
        print(f"结果已保存到 {filename}")

# ============================================================================
# 测试函数
# ============================================================================

def test_basic_control():
    """测试基本控制功能"""
    print("测试1: 基本高度控制")
    print("-" * 40)
    
    simulator = DroneSimulator()
    
    # 设置初始条件
    simulator.state.altitude = 1.0
    simulator.target_altitude = 5.0
    simulator.control_output.armed = True
    
    # 运行短时间模拟
    history = simulator.run_simulation(duration=10.0)
    
    # 分析结果
    final_altitude = history['altitude'][-1]
    target_altitude = history['target_altitude'][-1]
    altitude_error = abs(final_altitude - target_altitude)
    
    print(f"\n测试结果:")
    print(f"  最终高度: {final_altitude:.2f}m")
    print(f"  目标高度: {target_altitude:.2f}m")
    print(f"  高度误差: {altitude_error:.2f}m")
    
    if altitude_error < 0.5:
        print("  ✅ 高度控制测试通过")
        return True
    else:
        print("  ❌ 高度控制测试失败")
        return False

def test_attitude_control():
    """测试姿态控制功能"""
    print("\n测试2: 姿态控制")
    print("-" * 40)
    
    simulator = DroneSimulator()
    
    # 设置初始条件
    simulator.state.altitude = 5.0
    simulator.target_altitude = 5.0
    simulator.target_attitude.roll = 0.3  # 约17.2°
    simulator.target_attitude.pitch = -0.2  # 约-11.5°
    simulator.control_output.armed = True
    
    # 运行模拟
    history = simulator.run_simulation(duration=8.0)
    
    # 分析结果
    final_roll = history['roll'][-1]
    final_pitch = history['pitch'][-1]
    target_roll = math.degrees(simulator.target_attitude.roll)
    target_pitch = math.degrees(simulator.target_attitude.pitch)
    
    roll_error = abs(final_roll - target_roll)
    pitch_error = abs(final_pitch - target_pitch)
    
    print(f"\n测试结果:")
    print(f"  最终横滚: {final_roll:.1f}° (目标: {target_roll:.1f}°)")
    print(f"  最终俯仰: {final_pitch:.1f}° (目标: {target_pitch:.1f}°)")
    print(f"  横滚误差: {roll_error:.1f}°")
    print(f"  俯仰误差: {pitch_error:.1f}°")
    
    if roll_error < 5.0 and pitch_error < 5.0:
        print("  ✅ 姿态控制测试通过")
        return True
    else:
        print("  ❌ 姿态控制测试失败")
        return False

def test_safety_features():
    """测试安全功能"""
    print("\n测试3: 安全功能")
    print("-" * 40)
    
    simulator = DroneSimulator()
    
    # 测试1: 超过最大高度
    print("子测试1: 超过最大高度")
    simulator.state.altitude = simulator.config.max_altitude + 1.0
    simulator.target_altitude = simulator.config.max_altitude + 2.0
    simulator.control_output.armed = True
    
    # 运行几个控制周期
    for _ in range(100):
        simulator.run_control_cycle()
    
    if not simulator.control_output.armed:
        print("  ✅ 高度超限安全保护生效")
    else:
        print("  ❌ 高度超限安全保护未生效")
    
    # 测试2: 超过最大姿态角
    print("子测试2: 超过最大姿态角")
    simulator = DroneSimulator()  # 重置模拟器
    simulator.state.euler.roll = simulator.config.max_angle.roll + 0.1
    simulator.control_output.armed = True
    
    for _ in range(100):
        simulator.run_control_cycle()
    
    if not simulator.control_output.armed:
        print("  ✅ 姿态超限安全保护生效")
    else:
        print("  ❌ 姿态超限安全保护未生效")
    
    # 测试3: 电机输出超限
    print("子测试3: 电机输出超限")
    simulator = DroneSimulator()  # 重置模拟器
    simulator.control_output.motor1 = simulator.config.motor_max_pwm + 10.0
    simulator.control_output.armed = True
    
    for _ in range(100):
        simulator.run_control_cycle()
    
    if not simulator.control_output.armed:
        print("  ✅ 电机输出超限安全保护生效")
    else:
        print("  ❌ 电机输出超限安全保护未生效")
    
    return True

def test_pid_tuning():
    """测试PID参数调整"""
    print("\n测试4: PID参数调整")
    print("-" * 40)
    
    # 测试不同的PID参数
    pid_configs = [
        {"name": "默认参数", "kp": 3.0, "ki": 0.8, "kd": 0.2},
        {"name": "高增益", "kp": 5.0, "ki": 1.0, "kd": 0.5},
        {"name": "低增益", "kp": 1.0, "ki": 0.2, "kd": 0.05},
    ]
    
    results = []
    
    for config in pid_configs:
        print(f"\n测试配置: {config['name']}")
        
        simulator = DroneSimulator()
        
        # 更新PID参数
        simulator.roll_pid.kp = config['kp']
        simulator.roll_pid.ki = config['ki']
        simulator.roll_pid.kd = config['kd']
        
        simulator.pitch_pid.kp = config['kp']
        simulator.pitch_pid.ki = config['ki']
        simulator.pitch_pid.kd = config['kd']
        
        # 设置测试条件
        simulator.state.altitude = 1.0
        simulator.target_altitude = 5.0
        simulator.target_attitude.roll = 0.2
        simulator.control_output.armed = True
        
        # 运行短时间模拟
        history = simulator.run_simulation(duration=5.0)
        
        # 计算性能指标
        altitude_error = np.abs(np.array(history['altitude']) - np.array(history['target_altitude']))
        avg_error = np.mean(altitude_error[-100:])  # 最后100个样本
        
        roll_overshoot = max(history['roll']) - math.degrees(simulator.target_attitude.roll)
        
        results.append({
            "name": config['name'],
            "avg_error": avg_error,
            "roll_overshoot": roll_overshoot
        })
        
        print(f"  平均高度误差: {avg_error:.3f}m")
        print(f"  横滚超调量: {roll_overshoot:.1f}°")
    
    # 找出最佳配置
    best_config = min(results, key=lambda x: x['avg_error'])
    print(f"\n最佳配置: {best_config['name']} (平均误差: {best_config['avg_error']:.3f}m)")
    
    return True

def run_comprehensive_simulation():
    """运行综合模拟"""
    print("=" * 60)
    print("ESP32无人机飞控模拟器 - 综合测试")
    print("=" * 60)
    
    # 创建模拟器
    simulator = DroneSimulator()
    
    # 设置初始条件
    simulator.state.altitude = 0.5
    simulator.target_altitude = 5.0
    simulator.control_output.armed = True
    
    print("初始条件:")
    print(f"  初始高度: {simulator.state.altitude:.1f}m")
    print(f"  目标高度: {simulator.target_altitude:.1f}m")
    print(f"  无人机质量: {simulator.config.mass:.1f}kg")
    print(f"  悬停油门: {simulator.config.hover_throttle:.1f}%")
    print()
    
    # 运行模拟
    history = simulator.run_simulation(duration=30.0)
    
    # 性能分析
    print("\n性能分析:")
    print("-" * 40)
    
    # 高度跟踪性能
    altitude_error = np.abs(np.array(history['altitude']) - np.array(history['target_altitude']))
    max_alt_error = np.max(altitude_error)
    avg_alt_error = np.mean(altitude_error)
    std_alt_error = np.std(altitude_error)
    
    print(f"高度跟踪:")
    print(f"  最大误差: {max_alt_error:.3f}m")
    print(f"  平均误差: {avg_alt_error:.3f}m")
    print(f"  误差标准差: {std_alt_error:.3f}m")
    
    # 姿态稳定性
    roll_std = np.std(history['roll'])
    pitch_std = np.std(history['pitch'])
    yaw_std = np.std(history['yaw'])
    
    print(f"\n姿态稳定性:")
    print(f"  横滚标准差: {roll_std:.2f}°")
    print(f"  俯仰标准差: {pitch_std:.2f}°")
    print(f"  偏航标准差: {yaw_std:.2f}°")
    
    # 电机输出分析
    motor_outputs = np.array(history['motor_outputs'])
    avg_motor_output = np.mean(motor_outputs, axis=0)
    std_motor_output = np.std(motor_outputs, axis=0)
    
    print(f"\n电机输出:")
    for i in range(4):
        print(f"  电机{i+1}: 平均={avg_motor_output[i]:.1f}%, 标准差={std_motor_output[i]:.1f}%")
    
    # 能量消耗估算
    total_energy = np.sum(motor_outputs) * simulator.dt * 0.01  # 简化模型
    print(f"\n能量消耗估算: {total_energy:.1f} 能量单位")
    
    # 绘制结果
    print("\n生成图表...")
    simulator.plot_results(history)
    
    # 保存结果
    simulator.save_results(history)
    
    return history

# ============================================================================
# 主函数
# ============================================================================

def main():
    """主函数"""
    print("ESP32无人机飞控模拟器")
    print("=" * 60)
    
    # 运行单元测试
    print("运行单元测试...")
    print()
    
    test_results = []
    
    # 测试1: 基本高度控制
    test1_passed = test_basic_control()
    test_results.append(("基本高度控制", test1_passed))
    
    # 测试2: 姿态控制
    test2_passed = test_attitude_control()
    test_results.append(("姿态控制", test2_passed))
    
    # 测试3: 安全功能
    test3_passed = test_safety_features()
    test_results.append(("安全功能", test3_passed))
    
    # 测试4: PID参数调整
    test4_passed = test_pid_tuning()
    test_results.append(("PID参数调整", test4_passed))
    
    # 显示测试结果摘要
    print("\n" + "=" * 60)
    print("测试结果摘要")
    print("=" * 60)
    
    passed_count = 0
    for test_name, passed in test_results:
        status = "✅ 通过" if passed else "❌ 失败"
        print(f"{test_name:20} {status}")
        if passed:
            passed_count += 1
    
    print(f"\n通过率: {passed_count}/{len(test_results)} ({passed_count/len(test_results)*100:.0f}%)")
    
    if passed_count == len(test_results):
        print("\n所有测试通过! 开始综合模拟...")
        print()
        
        # 运行综合模拟
        run_comprehensive_simulation()
    else:
        print("\n部分测试失败，请检查代码问题。")
    
    print("\n模拟器运行完成!")

if __name__ == "__main__":
    # 检查依赖
    try:
        import numpy as np
        import matplotlib.pyplot as plt
    except ImportError as e:
        print(f"错误: 缺少依赖库 - {e}")
        print("请安装所需库: pip install numpy matplotlib")
        exit(1)
    
    main()
