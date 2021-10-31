#ifndef _RM_EXPORT_H_
#define _RM_EXPORT_H_

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#ifndef RM_EXPORT
#ifdef _WIN32
#define RM_EXPORT __declspec(dllexport)
#else
#define RM_EXPORT
#define __stdcall
#endif
#endif

RM_EXPORT void __stdcall motor_move(int motor_type, int direction, int step);
RM_EXPORT void __stdcall motor_stop(int motor_type);
RM_EXPORT void __stdcall motor_reset(int motor_type);
RM_EXPORT void __stdcall motor_clear(int motor_type);
RM_EXPORT void __stdcall light_motor_reset();
RM_EXPORT void __stdcall front_back_motor_reset();
RM_EXPORT void __stdcall left_right_motor_reset();
RM_EXPORT void __stdcall up_down_motor_reset();
RM_EXPORT void __stdcall all_motor_reset();
RM_EXPORT int __stdcall read_config(void*);
RM_EXPORT int __stdcall write_config(const void* config);
RM_EXPORT int __stdcall get_cam_num();
RM_EXPORT int __stdcall camera_thread(int width, int height, int hDC, int camNo);
RM_EXPORT int __stdcall start_thread(int camNo);
RM_EXPORT int __stdcall pause_thread(int camNo);
RM_EXPORT int __stdcall unpause_thread(int camNo);
RM_EXPORT int __stdcall stop_thread(int camNo);
RM_EXPORT int __stdcall save_image(char *savePath, int camNo);
RM_EXPORT int __stdcall copy_image(int camNo);
RM_EXPORT int __stdcall quit_process();

#ifdef __cplusplus
}
#endif // __cplusplus

#endif //_RM_EXPORT_H_
