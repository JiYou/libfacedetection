#ifndef _RM_EXPORT_BUTTON_H_
#define _RM_EXPORT_BUTTON_H_

#include "nce/camera.h"
#include "nce/locator.h"
#include "nce/motor.h"

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

#ifndef RM_EXPORT
#ifdef _WIN32
#define RM_EXPORT __declspec(dllexport)
#else
#define RM_EXPORT
#define __stdcall
#endif
#endif

/*
|ID_BUTTON （0x1000） | 0 | 0 | 0    |
|R_BUTTON （0x1001） | 0 | unlock = 0 / lock = 1 | 0    |
|UP_BUTTON （0x1002） | 0 / 1 | 0 | 0    |
|DOWN_BUTTON(0x1003) | 0 / 1 | 0 | 0    |
|MOTOR_RESET_BUTTON(0x1004) | 0 | 0 | 0    |
|CONFIG_BUTTON(0x1005) | 0 | 0 | 0    |
|L_BUTTON(0x1006) | 0 | 0 | 0    |
|FRONT_BUTTON(0x1007) | 0 / 1 | 0 | 0    |
|BACK_BUTTON(0x1008) | 0 / 1 | 0 | 0    |
|START_BUTTON(0x1009) | 0 | 0 | 0    |
|MEASURE_MODE_BUTTON(0x100a) | 0 | KPT = 0, K / R = 1, REF = 2 | 0    |
|CATARACT_BUTTON(0x100b) | 0 | 0 | 0    |
|FIX_TARGET_BUTTON(0x100c) | 0 | H = 0, L = 1 | 0    |
|FOG_BUTTON(0x100d) | 0 | T1 = 0, T3 = 1 | 0    |
|TARGET_IMG_BUTTON(0x100e) | 0 | 0 | 0    |
|PRINT_BUTTON(0x100f) | 0 | 0 | 0    |
|CLEAR_BUTTON(0x1010) | 0 | 0 | 0    |
|CORNEAR_DIA_BUTTON(0x1011) | 0 | 0 | 0    |
|AUTO_MANNUAL_BUTTON(0x1012) | 0 | AUTO = 0 / MANNUAL = 1 | 0 |
|TOUCH_BUTTON (0x1013)       |  0/1    | x坐标             | y 坐标|
*/
enum Button1st {
    ID_BUTTON = 0x1000,
    R_BUTTON,
    UP_BUTTON,
    DOWN_BUTTON,
    MOTOR_RESET_BUTTON,
    CONFIG_BUTTON,
    L_BUTTON,
    FRONT_BUTTON,
    BACK_BUTTON,
    START_BUTTON,
    MEASURE_MODE_BUTTON,
    CATARACT_BUTTON,
    FIX_TARGET_BUTTON,
    FOG_BUTTON,
    TARGET_IMG_BUTTON,
    PRINT_BUTTON,
    CLEAR_BUTTON,
    CORNEAR_DIA_BUTTON,
    AUTO_MANNUAL_BUTTON,
    TOUCH_BUTTON,
    // following are debug buttons
    // 第一行（Alignment 校准)
    CONF_1_RESET_BUTTON = 0x2100,
    CONF_1_START_BUTTON,
    CONF_1_STOP_BUTTON,
    CONF_1_LIGHT_BUTTON,
    CONF_1_REF_BUTTON,
    CONF_1_KER_BUTTON,
    // 第二行 (REF Setup 屈光度设置）
    CONF_2_RESET_BUTTON = 0x2200,
    CONF_2_MEAS_BUTTON,
    // 第三行(REF param set，屈光度参数设置)
    // 暂无
    // 第四行(KER setup 角膜曲率设置)
    CONF_4_RESET_BUTTON = 0x2400,
    CONF_4_MEAS_BUTTON,
    // 第五行(PD setup 瞳距设置)
    CONF_5_MEAS_BUTTON = 0x2500,
    CONF_5_RESET_BUTTON,
    CONF_5_R_BUTTON, //0x2502
    CONF_5_L_BUTTON, //0x2503
    // 第六行(Motor setup 马达设置)
    CONF_6_RESET_BUTTON = 0x2600,
    CONF_6_MEAS_BUTTON
};
/*
 * ui_call_backend
 * 前端调用后端的接口，或者是前端向后端发送事件。
 * args:
 * - MSG_SOURCE: 主要是指消息来源，比如是界面上的BUTTON
 *   那么就需要把界面上的BUTTON前后端用统一的编号
 *   比如
 *   constexpr int kMsgSourceMotorMoveButton = 0x1000;
 *
 * - MSG_CONTENT: 主要是指消息的事件类型
 *   比如按钮按下
 *   constexpr int kMsgSourceMotorMoveButtonPressDown = 0x1001;
 * - DATA1:
 * - DATA2:
 *   可以附加的数据项，比如触摸屏按下时，可能需要把坐标发送到后端来。
 *   DATA1可能就是行坐标
 *   DATA2可能就是列坐标
 */
RM_EXPORT int __stdcall ui_call_backend(const int MSG_SOURCE,
                                        const int MSG_CONTENT, const int DATA1,
                                        const int DATA2);

// 提供给UI的结构体，需要如下方式封装，并保存在参数指针的函数中。
#ifdef _WIN32
#pragma pack(push)  // 保持对齐方式
#pragma pack(1)     // 设定1字节对齐
#endif

struct RefData {
    float ref_s;  // 屈光度 对应 S: S（Spherical）:球镜-近视（带负号）或者远视度数（带正号）
    float ref_c;  // 散光  对应 C: C（Cylindrical）:柱镜-散光度数（默认为负值）
    float ref_ax;  //轴位 对应 A: A（Axls）：轴位-散光轴位
    int motor_step; //用于求瞳距
};

struct KrtData {
    float krt_r1;  //长轴
    float krt_r2;  //短轴
    float krt_ax;  //轴位
    int motor_step; //用于求瞳距
};

struct R_K_Data {
    struct RefData ref;
    struct KrtData krt;
    int motor_step; //用于求瞳距
};

RM_EXPORT int __stdcall ops_running_over(const int ops_id);

RM_EXPORT int __stdcall measure_ref(const int MSG_SOURCE,
                                    struct RefData *data);

RM_EXPORT int __stdcall measure_krt(const int MSG_SOURCE,
                                    struct KrtData *data);

RM_EXPORT int __stdcall measure_ref_krt(const int MSG_SOURCE,
                                        struct R_K_Data *data);

/**
 * 函数功能：
 * 
 *     当按下这个按钮，会在新出来的界面上显示图片
 * 
 * 函数参数：
 *      - hDC是用来画图的新接口
 *      - image_source用来指示显示测量过的左眼的图片还是右眼的图片
 *      - idx表示用来显示缓冲区中的哪一张图片
 * 
 * 返回值:
 *      - 0:   表示成功
 *      - 负数: 表示失败
 */
RM_EXPORT int __stdcall start_show_measured_image(const int image_source,
                                                  const int idx);

/**
 * 函数功能
 *
 *      退出图片显示模式，仍然显示摄像头来的图片
 *
 * 参数：
 *      - data: 暂时用不到
 *
 * 返回值:
 *      - 0:    表示操作成功
 *      - 负数： 表示操作失败
 */
RM_EXPORT int __stdcall end_show_measured_image(const int data);

struct CurMotorPosData {
    int lr_motor_step;
    int ud_motor_step;
    int fb_motor_step;
};

RM_EXPORT int __stdcall get_all_motor_cur_pos(const int MSG_SOURCE,
        struct CurMotorPosData *data);

#ifdef _WIN32
#pragma pack(pop)  // 恢复对齐方式
#endif

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  //_RM_EXPORT_BUTTON_H_
