// lights_controller.cpp
// 灯光控制器实现 - 管理按钮灯和三色灯
// ============================================================================

#include "lights_controller.hpp"
#include <cstdio>
#include <chrono>

// ============================================================================
// 内部状态变量
// ============================================================================

// 按钮灯状态
static bool g_start_light_on = false;
static bool g_reset_light_on = false;
static bool g_pause_light_on = false;

// 按钮上次状态（用于边沿检测）
static bool g_last_start_button = false;
static bool g_last_reset_button = false;
static bool g_last_pause_button = false;

// 按钮灯DO上次写入值（用于变化检测）
static bool g_last_do800 = false;
static bool g_last_do801 = false;
static bool g_last_do802 = false;

// 复位按钮3秒确认计时
static bool g_reset_button_held = false;
static auto g_reset_press_start_time = std::chrono::steady_clock::now();
static const int RESET_CONFIRM_MS = 3000;  // 3秒确认时间

// 三色灯状态
static TricolorLightState g_tricolor_state = LIGHT_OFF;

// 闪烁控制
static auto g_last_toggle_time = std::chrono::steady_clock::now();
static bool g_blink_state = false;
static bool g_last_yellow_do = false;
static bool g_last_green_do = false;
static bool g_last_buzzer_do = false;

// 闪烁周期(ms)
static const int BLINK_PERIOD_MS = 500;

// ============================================================================
// 外部全局变量声明
// ============================================================================

extern std::atomic<bool> g_start_button_pressed;
extern std::atomic<bool> g_reset_button_pressed;
extern std::atomic<bool> g_pause_button_pressed;

// ============================================================================
// 内部辅助函数
// ============================================================================

// 写入按钮灯DO（带变化检测）
static void write_button_light(int address, bool state, bool& last_state) {
    if (state != last_state) {
        write_single_do_signal(address, state);
        last_state = state;
    }
}

// ============================================================================
// 接口实现
// ============================================================================

void init_lights_controller() {
    g_start_light_on = false;
    g_reset_light_on = false;
    g_pause_light_on = false;
    g_tricolor_state = LIGHT_OFF;
    g_blink_state = false;
    
    // 初始化DO输出
    write_single_do_signal(800, false);  // 启动灯
    write_single_do_signal(801, false);  // 复位灯
    write_single_do_signal(802, false);  // 暂停灯
    write_single_do_signal(803, false);  // 蜂鸣器
    write_single_do_signal(805, false);  // 黄灯
    write_single_do_signal(806, false);  // 绿灯
    
    printf("[LightsController] 灯光控制器初始化完成\n");
}

void update_button_lights(bool start_btn, bool reset_btn, bool pause_btn) {
    // 边沿检测
    bool start_rising = start_btn && !g_last_start_button;
    bool reset_rising = reset_btn && !g_last_reset_button;
    bool pause_rising = pause_btn && !g_last_pause_button;
    
    // 启动按钮：点亮启动灯，熄灭其他灯，绿灯常亮，停止蜂鸣器
    if (start_rising) {
        g_start_light_on = true;
        g_reset_light_on = false;
        g_pause_light_on = false;
        g_tricolor_state = LIGHT_GREEN_ON;  // 绿灯常亮
        g_start_button_pressed.store(true);
        // 取消复位计时（如果正在进行）
        g_reset_button_held = false;
        // 确保蜂鸣器停止
        if (g_last_buzzer_do) {
            write_single_do_signal(803, false);
            g_last_buzzer_do = false;
        }
        printf("[Lights] 启动按钮触发，启动灯亮起，绿灯常亮，蜂鸣器停\n");
    }
    
    // 复位按钮：按住3秒确认，点亮复位灯，启动黄灯闪烁+蜂鸣器
    if (reset_btn && !g_last_reset_button) {
        // 复位按钮刚按下，开始计时
        g_reset_button_held = true;
        g_reset_press_start_time = std::chrono::steady_clock::now();
        printf("[Lights] 复位按钮按下，开始3秒计时...\n");
    } else if (!reset_btn && g_last_reset_button) {
        // 复位按钮释放，如果未满3秒则取消
        if (g_reset_button_held) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - g_reset_press_start_time).count();
            if (elapsed < RESET_CONFIRM_MS) {
                printf("[Lights] 复位按钮提前释放（%ld ms < 3000 ms），取消复位\n", elapsed);
            }
            g_reset_button_held = false;
        }
    }
    
    // 检查是否满足3秒确认条件
    if (g_reset_button_held) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - g_reset_press_start_time).count();
        if (elapsed >= RESET_CONFIRM_MS && !g_reset_light_on) {
            // 满3秒，触发复位
            g_reset_light_on = true;
            g_start_light_on = false;
            g_pause_light_on = false;
            g_tricolor_state = LIGHT_YELLOW_BLINK;
            g_reset_button_pressed.store(true);
            printf("[Lights] 复位按钮确认（3秒），复位灯亮起，黄灯闪烁，蜂鸣器响\n");
        }
    }
    
    // 暂停按钮：点亮暂停灯，熄灭其他灯，停止三色灯和蜂鸣器
    if (pause_rising) {
        g_pause_light_on = true;
        g_start_light_on = false;
        g_reset_light_on = false;
        g_tricolor_state = LIGHT_OFF;
        g_pause_button_pressed.store(true);
        // 取消复位计时（如果正在进行）
        g_reset_button_held = false;
        // 确保蜂鸣器停止
        if (g_last_buzzer_do) {
            write_single_do_signal(803, false);
            g_last_buzzer_do = false;
        }
        printf("[Lights] 暂停按钮触发，暂停灯亮起，三色灯停止，蜂鸣器停\n");
    }
    
    // 更新上次状态
    g_last_start_button = start_btn;
    g_last_reset_button = reset_btn;
    g_last_pause_button = pause_btn;
    
    // 写入按钮灯DO（只在状态变化时写入）
    write_button_light(800, g_start_light_on, g_last_do800);
    write_button_light(801, g_reset_light_on, g_last_do801);
    write_button_light(802, g_pause_light_on, g_last_do802);
}

void update_tricolor_lights() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - g_last_toggle_time).count();
    
    // 每周期切换一次闪烁状态
    if (elapsed >= BLINK_PERIOD_MS) {
        g_blink_state = !g_blink_state;
        g_last_toggle_time = now;
    }
    
    // 根据当前状态控制三色灯和蜂鸣器
    bool yellow_should_on = false;
    bool green_should_on = false;
    bool buzzer_should_on = false;  // 蜂鸣器
    
    switch (g_tricolor_state) {
        case LIGHT_YELLOW_BLINK:
            yellow_should_on = g_blink_state;
            green_should_on = false;
            buzzer_should_on = true;  // 黄灯闪烁时蜂鸣器响
            break;
        case LIGHT_GREEN_BLINK:
            yellow_should_on = false;
            green_should_on = g_blink_state;
            buzzer_should_on = false;  // 绿灯闪烁时蜂鸣器停
            break;
        case LIGHT_GREEN_ON:
            yellow_should_on = false;
            green_should_on = true;  // 绿灯常亮
            buzzer_should_on = false;
            break;
        case LIGHT_OFF:
        default:
            yellow_should_on = false;
            green_should_on = false;
            buzzer_should_on = false;
            break;
    }
    
    // 只在状态变化时写入DO（减少Modbus通信）
    if (yellow_should_on != g_last_yellow_do) {
        write_single_do_signal(805, yellow_should_on);  // 黄灯
        g_last_yellow_do = yellow_should_on;
    }
    if (green_should_on != g_last_green_do) {
        write_single_do_signal(806, green_should_on);   // 绿灯
        g_last_green_do = green_should_on;
    }
    if (buzzer_should_on != g_last_buzzer_do) {
        write_single_do_signal(803, buzzer_should_on);  // 蜂鸣器 M803
        g_last_buzzer_do = buzzer_should_on;
    }
}

void set_tricolor_state(TricolorLightState state) {
    if (g_tricolor_state != state) {
        g_tricolor_state = state;
        printf("[Lights] 三色灯状态切换: %d\n", state);
    }
}

TricolorLightState get_tricolor_state() {
    return g_tricolor_state;
}

void notify_system_ready() {
    // 从黄灯闪烁切换到绿灯闪烁，停止蜂鸣器
    if (g_tricolor_state == LIGHT_YELLOW_BLINK) {
        g_tricolor_state = LIGHT_GREEN_BLINK;
        if (g_reset_light_on) {
            g_reset_light_on = false;
            write_button_light(801, false, g_last_do801);
        }
        // 确保蜂鸣器停止
        if (g_last_buzzer_do) {
            write_single_do_signal(803, false);
            g_last_buzzer_do = false;
        }
        printf("[Lights] 系统就绪，黄灯停闪，绿灯闪烁，蜂鸣器停\n");
    }
}

bool is_start_light_on() {
    return g_start_light_on;
}

bool is_reset_light_on() {
    return g_reset_light_on;
}

bool is_pause_light_on() {
    return g_pause_light_on;
}
