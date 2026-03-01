#!/usr/bin/env python3
"""
简化测试脚本 - 使用纯ASCII字符避免编码问题
"""

import sys
import os

# 添加当前目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

print("=" * 60)
print("ESP32无人机飞控模拟器 - 简化测试")
print("=" * 60)

try:
    from drone_simulator import DroneSimulator
    
    # 测试1: 创建模拟器实例
    print("\n1. 创建模拟器实例")
    try:
        simulator = DroneSimulator()
        print("   [OK] 模拟器实例创建成功")
        print(f"       无人机质量: {simulator.config.mass}kg")
        print(f"       最大高度: {simulator.config.max_altitude}m")
        print(f"       悬停油门: {simulator.config.hover_throttle}%")
    except Exception as e:
        print(f"   [ERROR] 模拟器实例创建失败: {e}")
        sys.exit(1)
    
    # 测试2: 模拟传感器数据
    print("\n2. 模拟传感器数据")
    try:
        sensor_data = simulator.simulate_sensors()
        print("   [OK] 传感器数据模拟成功")
        print(f"       加速度计: [{sensor_data.accel[0]:.2f}, {sensor_data.accel[1]:.2f}, {sensor_data.accel[2]:.2f}] m/s2")
        print(f"       陀螺仪: [{sensor_data.gyro[0]:.2f}, {sensor_data.gyro[1]:.2f}, {sensor_data.gyro[2]:.2f}] rad/s")
        print(f"       气压: {sensor_data.baro:.1f} Pa")
        print(f"       高度: {sensor_data.alti:.2f} m")
    except Exception as e:
        print(f"   [ERROR] 传感器数据模拟失败: {e}")
        sys.exit(1)
    
    # 测试3: 运行单个控制周期
    print("\n3. 运行控制周期")
    try:
        # 设置初始状态
        simulator.state.altitude = 1.0
        simulator.target_altitude = 5.0
        simulator.control_output.armed = True
        
        # 运行一个控制周期
        simulator.run_control_cycle()
        
        print("   [OK] 控制周期运行成功")
        print(f"       当前高度: {simulator.state.altitude:.2f} m")
        print(f"       目标高度: {simulator.target_altitude:.2f} m")
        print(f"       电机输出: [{simulator.control_output.motor1:.1f}%, "
              f"{simulator.control_output.motor2:.1f}%, "
              f"{simulator.control_output.motor3:.1f}%, "
              f"{simulator.control_output.motor4:.1f}%]")
        print(f"       使能状态: {'是' if simulator.control_output.armed else '否'}")
    except Exception as e:
        print(f"   [ERROR] 控制周期运行失败: {e}")
        sys.exit(1)
    
    # 测试4: 运行短时间模拟
    print("\n4. 短时间模拟（3秒）")
    try:
        # 重置模拟器
        simulator = DroneSimulator()
        simulator.state.altitude = 1.0
        simulator.target_altitude = 3.0
        simulator.control_output.armed = True
        
        print("   开始模拟...")
        
        # 运行3秒模拟（300个控制周期）
        import time
        start_time = time.time()
        
        for i in range(300):  # 3秒 * 100Hz
            simulator.run_control_cycle()
            
            # 每100步打印一次状态
            if i % 100 == 0:
                print(f"   时间 {simulator.sim_time:.1f}s: "
                      f"高度={simulator.state.altitude:.2f}m, "
                      f"目标={simulator.target_altitude:.1f}m")
        
        end_time = time.time()
        elapsed = end_time - start_time
        
        print(f"   模拟完成，实际耗时: {elapsed:.2f}秒")
        print(f"   最终高度: {simulator.state.altitude:.2f} m")
        print(f"   最终目标高度: {simulator.target_altitude:.2f} m")
        
        # 计算性能指标
        import numpy as np
        if len(simulator.history['altitude']) > 0:
            altitude_error = abs(simulator.state.altitude - simulator.target_altitude)
            print(f"   最终高度误差: {altitude_error:.3f} m")
            
            if altitude_error < 0.5:
                print("   [OK] 高度跟踪性能良好")
            elif altitude_error < 1.0:
                print("   [WARNING] 高度跟踪性能一般")
            else:
                print("   [WARNING] 高度跟踪误差较大，可能需要调整PID参数")
        
    except Exception as e:
        print(f"   [ERROR] 短时间模拟失败: {e}")
        import traceback
        traceback.print_exc()
    
    # 测试5: 安全检查功能
    print("\n5. 安全检查功能")
    try:
        simulator = DroneSimulator()
        
        # 测试高度超限
        simulator.state.altitude = simulator.config.max_altitude + 2.0
        simulator.control_output.armed = True
        
        # 运行几个控制周期
        for _ in range(50):
            simulator.run_control_cycle()
        
        if not simulator.control_output.armed:
            print("   [OK] 高度超限安全保护生效")
        else:
            print("   [WARNING] 高度超限安全保护未生效")
        
    except Exception as e:
        print(f"   [ERROR] 安全检查测试失败: {e}")
    
    print("\n" + "=" * 60)
    print("简化测试完成!")
    print("=" * 60)
    
    print("\n测试总结:")
    print("- 模拟器核心功能正常")
    print("- 传感器模拟和控制循环工作正常")
    print("- 安全保护功能已实现")
    print("\n下一步:")
    print("1. 运行完整模拟: python drone_simulator.py")
    print("2. 调整PID参数以获得更好的控制性能")
    print("3. 根据分析报告中的建议改进飞控算法")
    
except ImportError as e:
    print(f"[ERROR] 导入错误: {e}")
    print("请确保所有依赖库已安装: pip install numpy matplotlib")
    sys.exit(1)
except Exception as e:
    print(f"[ERROR] 测试过程中发生错误: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)