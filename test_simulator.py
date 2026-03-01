#!/usr/bin/env python3
"""
简化测试脚本 - 验证模拟器基本功能
"""

import sys
import os

# 添加当前目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from drone_simulator import DroneSimulator, test_basic_control, test_attitude_control
    
    print("=" * 60)
    print("ESP32无人机飞控模拟器 - 简化测试")
    print("=" * 60)
    
    # 测试1: 创建模拟器实例
    print("\n测试1: 创建模拟器实例")
    try:
        simulator = DroneSimulator()
        print("  [OK] 模拟器实例创建成功")
        print(f"    无人机质量: {simulator.config.mass}kg")
        print(f"    最大高度: {simulator.config.max_altitude}m")
        print(f"    悬停油门: {simulator.config.hover_throttle}%")
    except Exception as e:
        print(f"  [ERROR] 模拟器实例创建失败: {e}")
        sys.exit(1)
    
    # 测试2: 模拟传感器数据
    print("\n测试2: 模拟传感器数据")
    try:
        sensor_data = simulator.simulate_sensors()
        print("  ✅ 传感器数据模拟成功")
        print(f"    加速度计: [{sensor_data.accel[0]:.2f}, {sensor_data.accel[1]:.2f}, {sensor_data.accel[2]:.2f}] m/s²")
        print(f"    陀螺仪: [{sensor_data.gyro[0]:.2f}, {sensor_data.gyro[1]:.2f}, {sensor_data.gyro[2]:.2f}] rad/s")
        print(f"    气压: {sensor_data.baro:.1f} Pa")
        print(f"    高度: {sensor_data.alti:.2f} m")
    except Exception as e:
        print(f"  ❌ 传感器数据模拟失败: {e}")
        sys.exit(1)
    
    # 测试3: 运行单个控制周期
    print("\n测试3: 运行控制周期")
    try:
        # 设置初始状态
        simulator.state.altitude = 1.0
        simulator.target_altitude = 5.0
        simulator.control_output.armed = True
        
        # 运行一个控制周期
        simulator.run_control_cycle()
        
        print("  ✅ 控制周期运行成功")
        print(f"    当前高度: {simulator.state.altitude:.2f} m")
        print(f"    目标高度: {simulator.target_altitude:.2f} m")
        print(f"    电机输出: [{simulator.control_output.motor1:.1f}%, "
              f"{simulator.control_output.motor2:.1f}%, "
              f"{simulator.control_output.motor3:.1f}%, "
              f"{simulator.control_output.motor4:.1f}%]")
        print(f"    使能状态: {'是' if simulator.control_output.armed else '否'}")
    except Exception as e:
        print(f"  ❌ 控制周期运行失败: {e}")
        sys.exit(1)
    
    # 测试4: 基本高度控制测试
    print("\n测试4: 基本高度控制")
    try:
        passed = test_basic_control()
        if passed:
            print("  ✅ 基本高度控制测试通过")
        else:
            print("  ⚠️  基本高度控制测试未通过（可能是参数需要调整）")
    except Exception as e:
        print(f"  ❌ 基本高度控制测试失败: {e}")
    
    # 测试5: 姿态控制测试
    print("\n测试5: 姿态控制")
    try:
        passed = test_attitude_control()
        if passed:
            print("  ✅ 姿态控制测试通过")
        else:
            print("  ⚠️  姿态控制测试未通过（可能是参数需要调整）")
    except Exception as e:
        print(f"  ❌ 姿态控制测试失败: {e}")
    
    # 测试6: 运行短时间模拟
    print("\n测试6: 短时间模拟（5秒）")
    try:
        # 重置模拟器
        simulator = DroneSimulator()
        simulator.state.altitude = 1.0
        simulator.target_altitude = 3.0
        simulator.control_output.armed = True
        
        # 运行5秒模拟
        history = simulator.run_simulation(duration=5.0)
        
        print("  ✅ 短时间模拟成功")
        print(f"    模拟步数: {len(history['time'])}")
        print(f"    最终高度: {history['altitude'][-1]:.2f} m")
        print(f"    最终目标高度: {history['target_altitude'][-1]:.2f} m")
        
        # 计算性能指标
        import numpy as np
        altitude_error = np.abs(np.array(history['altitude']) - np.array(history['target_altitude']))
        avg_error = np.mean(altitude_error)
        
        print(f"    平均高度误差: {avg_error:.3f} m")
        
        if avg_error < 1.0:
            print("  ✅ 高度跟踪性能可接受")
        else:
            print("  ⚠️  高度跟踪误差较大，可能需要调整PID参数")
        
    except Exception as e:
        print(f"  ❌ 短时间模拟失败: {e}")
        import traceback
        traceback.print_exc()
    
    print("\n" + "=" * 60)
    print("简化测试完成!")
    print("=" * 60)
    
    print("\n建议:")
    print("1. 如需完整测试，请运行 'python drone_simulator.py'")
    print("2. 如需调整PID参数，请修改DroneSimulator类中的PID控制器配置")
    print("3. 如需查看详细图表，请运行综合模拟")
    
except ImportError as e:
    print(f"导入错误: {e}")
    print("请确保所有依赖库已安装: pip install numpy matplotlib")
    sys.exit(1)
except Exception as e:
    print(f"测试过程中发生错误: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)