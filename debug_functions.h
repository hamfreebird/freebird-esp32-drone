// ============================================================================
// 调试和监控功能头文件
// ============================================================================

#ifndef DEBUG_FUNCTIONS_H
#define DEBUG_FUNCTIONS_H

#include <Arduino.h>

// 性能监控结构体
typedef struct {
    uint32_t loop_count;           // 循环计数
    uint32_t last_print_time;      // 上次打印时间
    float loop_frequency;          // 循环频率 (Hz)
    float max_loop_time;           // 最大循环时间 (ms)
    float min_loop_time;           // 最小循环时间 (ms)
    float avg_loop_time;           // 平均循环时间 (ms)
    uint32_t safety_violations;    // 安全检查违规次数
    uint32_t sensor_errors;        // 传感器错误次数
    uint32_t communication_errors; // 通信错误次数
} performance_monitor_t;

// 飞行数据记录结构体
typedef struct {
    uint32_t timestamp;           // 时间戳 (ms)
    sensor_data_t sensor_data;    // 传感器数据
    drone_state_t drone_state;    // 无人机状态
    control_output_t control_output; // 控制输出
    euler_angles_t target_attitude; // 目标姿态
    float target_altitude;        // 目标高度
} flight_data_record_t;

// 全局变量声明
extern performance_monitor_t perf_monitor;
extern flight_data_record_t flight_records[];
extern uint16_t flight_record_index;
extern bool recording_enabled;

// 函数声明
void initPerformanceMonitor();
void updatePerformanceMonitor(uint32_t loop_start_time);
void printPerformanceStats();
void startFlightDataRecording();
void stopFlightDataRecording();
void recordFlightData();
void exportFlightData();
void systemDiagnostics();
void emergencyLanding();

#endif // DEBUG_FUNCTIONS_H