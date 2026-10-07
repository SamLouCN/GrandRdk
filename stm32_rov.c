/**
 * STM32F411CEU6 ROV控制固件  核心逻辑
 * 功能: 串口解析指令 + 推进器混控 + LED PWM + 遥测回传 + PID 增益接收
 *
 * v3.1 变更:
 *   - 推进器数量 6 -> 12 (MOTOR_COUNT = 12)
 *   - 遥测扩展: 追加 加速度 ax/ay/az 与 高度 alt, 且上报 12 路推进器油门
 *   - 新增 PID 调试帧 $PID,ch,p,i,d# 解析, 每通道独立 Kp/Ki/Kd
 *
 * 将此文件中的函数集成到你的 HAL 工程中:
 *   - 在 main.c 的 while(1) 中调用 ROV_MainLoop()
 *   - USART中断中调用 ROV_UART_RxHandler(byte)
 *   - 需要开启: USART1(或2), TIM PWM, 以及你的IMU驱动
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* ==================== 硬件抽象接口(需用户实现) ==================== */
extern void HAL_Set_Thruster_PWM(int motor_id, int pwm);  // 设置推进器PWM
extern void HAL_Set_LED_PWM(int ch, int pwm);              // 设置LED PWM
extern void HAL_UART_SendString(const char* str);           // 串口发送字符串
extern float HAL_IMU_GetRoll(void);                         // 读取IMU Roll
extern float HAL_IMU_GetPitch(void);
extern float HAL_IMU_GetYaw(void);
extern float HAL_IMU_GetGyroX(void);
extern float HAL_IMU_GetGyroY(void);
extern float HAL_IMU_GetGyroZ(void);
/* v3.1 新增: 加速度与高度 */
extern float HAL_IMU_GetAccX(void);
extern float HAL_IMU_GetAccY(void);
extern float HAL_IMU_GetAccZ(void);
extern float HAL_Altitude_Get(void);                        // 高度(气压计/水深换算), 无则返回深度
extern float HAL_Depth_Get(void);                           // 深度传感器

/* ==================== 推进器ID定义 (12路) ==================== */
#define MOTOR_FL  0   // 前左
#define MOTOR_FR  1   // 前右
#define MOTOR_RL  2   // 后左
#define MOTOR_RR  3   // 后右
#define MOTOR_VL  4   // 垂直左
#define MOTOR_VR  5   // 垂直右
#define MOTOR_6   6   // 扩展水平(模板占位, 按实际布局修改)
#define MOTOR_7   7
#define MOTOR_8   8
#define MOTOR_9   9
#define MOTOR_10 10   // 扩展垂直(模板占位)
#define MOTOR_11 11
#define MOTOR_COUNT 12
#define PID_CHANNELS 12   // 与推进器数量一致

#define PWM_CENTER 1500  // ESC中位
#define PWM_RANGE  400   // 最大偏移量
#define PWM_MIN    1100
#define PWM_MAX    1900
#define LED_PWM_MAX 1000

/* ==================== 指令变量 ==================== */
static float cmd_surge = 0.0f;
static float cmd_sway  = 0.0f;
static float cmd_heave = 0.0f;
static float cmd_yaw   = 0.0f;
static int   cmd_led1  = 0;
static int   cmd_led2  = 0;

/* ==================== 推进器输出 & PID 增益 ==================== */
static float thr_out[MOTOR_COUNT];                 // 每路推进器油门 [-1,1]
static float pid_p[PID_CHANNELS], pid_i[PID_CHANNELS], pid_d[PID_CHANNELS];

/* ==================== 串口接收缓冲 ==================== */
#define RX_BUF_SIZE 160
static char rx_buf[RX_BUF_SIZE];
static int  rx_idx = 0;
static volatile int cmd_ready = 0;

/* ==================== 安全看门狗 ==================== */
static volatile uint32_t last_cmd_tick = 0;
#define CMD_TIMEOUT_MS 1000  // 1秒无指令则急停

extern uint32_t HAL_GetTick(void);

/* ==================== 串口接收(中断中调用) ==================== */
void ROV_UART_RxHandler(uint8_t byte) {
    if (byte == '$') {
        rx_idx = 0;
        rx_buf[rx_idx++] = byte;
    } else if (rx_idx > 0 && rx_idx < RX_BUF_SIZE - 1) {
        rx_buf[rx_idx++] = byte;
        if (byte == '#') {
            rx_buf[rx_idx] = '\0';
            cmd_ready = 1;
        }
    } else {
        rx_idx = 0;  // 溢出重置
    }
}

/* ==================== 解析指令 ==================== */
static void parse_command(void) {
    if (!cmd_ready) return;
    cmd_ready = 0;

    if (rx_buf[0] != '$' || rx_buf[rx_idx - 1] != '#') return;

    /* --- v3.1: PID 参数帧 $PID,ch,p,i,d# --- */
    if (strncmp(rx_buf, "$PID,", 5) == 0) {
        int ch; float p, i, d;
        int n = sscanf(rx_buf, "$PID,%d,%f,%f,%f#", &ch, &p, &i, &d);
        if (n == 4 && ch >= 0 && ch < PID_CHANNELS) {
            pid_p[ch] = p; pid_i[ch] = i; pid_d[ch] = d;
        }
        return;
    }

    /* --- 运动指令帧 $CMD,... --- */
    if (strncmp(rx_buf, "$CMD,", 5) != 0) return;
    float s, sw, h, y;
    int l1, l2;
    int n = sscanf(rx_buf, "$CMD,%f,%f,%f,%f,%d,%d#",
                   &s, &sw, &h, &y, &l1, &l2);
    if (n == 6) {
        cmd_surge = s; cmd_sway = sw; cmd_heave = h; cmd_yaw = y;
        cmd_led1 = l1; cmd_led2 = l2;
        last_cmd_tick = HAL_GetTick();
    }
}

/* ==================== 限幅工具 ==================== */
static int clamp_pwm(int val) {
    if (val < PWM_MIN) return PWM_MIN;
    if (val > PWM_MAX) return PWM_MAX;
    return val;
}

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ==================== 推进器混控 (12路, 模板) ==================== */
static void mixer_execute(void) {
    // 安全检查: 超时急停
    if (HAL_GetTick() - last_cmd_tick > CMD_TIMEOUT_MS) {
        cmd_surge = cmd_sway = cmd_heave = cmd_yaw = 0.0f;
    }

    float s  = clampf(cmd_surge, -1.0f, 1.0f);
    float sw = clampf(cmd_sway,  -1.0f, 1.0f);
    float h  = clampf(cmd_heave, -1.0f, 1.0f);
    float y  = clampf(cmd_yaw,   -1.0f, 1.0f);

    /*
     * X型布局混控矩阵 (8水平 + 4垂直; 模板, 按实际布局修改):
     *   T1  FL = surge + sway + yaw
     *   T2  FR = surge - sway - yaw
     *   T3  RL = surge - sway + yaw
     *   T4  RR = surge + sway - yaw
     *   T5~T8 扩展水平(占位, 复用同式)
     *   T9  垂直左 = heave
     *   T10 垂直右 = heave
     *   T11 垂直前 = heave
     *   T12 垂直后 = heave
     */
    thr_out[0]  = s + sw + y;
    thr_out[1]  = s - sw - y;
    thr_out[2]  = s - sw + y;
    thr_out[3]  = s + sw - y;
    thr_out[4]  = s + sw + y;   // 扩展水平(占位)
    thr_out[5]  = s - sw - y;   // 扩展水平(占位)
    thr_out[6]  = s - sw + y;   // 扩展水平(占位)
    thr_out[7]  = s + sw - y;   // 扩展水平(占位)
    thr_out[8]  = h;   // 垂直前(占位)
    thr_out[9]  = h;
    thr_out[10] = h;
    thr_out[11] = h;

    // 归一化: 如果任何通道超过1.0, 等比缩放(仅对前8水平通道)
    float mx = 0.0f;
    for (int i = 0; i < 8; i++) { if (fabsf(thr_out[i]) > mx) mx = fabsf(thr_out[i]); }
    if (mx > 1.0f) {
        for (int i = 0; i < 8; i++) thr_out[i] /= mx;
    }
    // 垂直通道限幅
    for (int i = 8; i < MOTOR_COUNT; i++) thr_out[i] = clampf(thr_out[i], -1.0f, 1.0f);

    // 输出 PWM (模板: 未接入 PID 闭环, 直接油门->PWM)
    for (int i = 0; i < MOTOR_COUNT; i++) {
        thr_out[i] = clampf(thr_out[i], -1.0f, 1.0f);
        HAL_Set_Thruster_PWM(i, clamp_pwm(PWM_CENTER + (int)(thr_out[i] * PWM_RANGE)));
    }

    // LED
    int led1_pwm = (cmd_led1 * LED_PWM_MAX) / 100;
    int led2_pwm = (cmd_led2 * LED_PWM_MAX) / 100;
    HAL_Set_LED_PWM(0, led1_pwm);
    HAL_Set_LED_PWM(1, led2_pwm);
}

/* ==================== 遥测发送 (v3.1, 41字段) ==================== */
static uint32_t telem_last = 0;
#define TELEM_INTERVAL_MS 100  // 10Hz遥测

static void send_telemetry(void) {
    if (HAL_GetTick() - telem_last < TELEM_INTERVAL_MS) return;
    telem_last = HAL_GetTick();

    char buf[512];
    /* 格式: $TEL,10实际,10目标,5电池,12推进器,ax,ay,az,alt#
     * 10实际: roll,pitch,yaw,gx,gy,gz,depth,vx,vy,vz
     * 10目标: t_roll~t_vz (模板填 0)
     * 5电池 : batt_v,batt_a,batt_soc,batt_temp,cabin_temp (模板填 0)
     */
    snprintf(buf, sizeof(buf),
        "$TEL,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,0.00,0.00,0.00,"
        "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,"
        "%f,%f,%f,%f,%f,"
        "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,"
        "%.2f,%.2f,%.2f,%.2f#\r\n",
        HAL_IMU_GetRoll(), HAL_IMU_GetPitch(), HAL_IMU_GetYaw(),
        HAL_IMU_GetGyroX(), HAL_IMU_GetGyroY(), HAL_IMU_GetGyroZ(),
        HAL_Depth_Get(),
        /* 10 目标(模板0) */
        0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
        /* 5 电池(模板0) */
        0.0f,0.0f,0.0f,0.0f,0.0f,
        /* 12 推进器 */
        thr_out[0],thr_out[1],thr_out[2],thr_out[3],thr_out[4],thr_out[5],
        thr_out[6],thr_out[7],thr_out[8],thr_out[9],thr_out[10],thr_out[11],
        /* v3.1 加速度/高度 */
        HAL_IMU_GetAccX(), HAL_IMU_GetAccY(), HAL_IMU_GetAccZ(), HAL_Altitude_Get());
    HAL_UART_SendString(buf);
}

/* ==================== 主循环(在while(1)中调用) ==================== */
void ROV_MainLoop(void) {
    parse_command();
    mixer_execute();
    send_telemetry();
}
