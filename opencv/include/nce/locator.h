#ifndef _RM_LOCATOR_H_
#define _RM_LOCATOR_H_

#define _DEBUG_ true

#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include "nce/camera.h"
#include "nce/export.h"
#include "nce/motor.h"
#include "nce/util.h"
#include "nce/line2Dup.h"

namespace RM {
// begin of namespace RM

class ClarityLocator : public ImageOperations {
  public:
    ClarityLocator(std::string s) : name_(s) {
        front_back_motor_ =
            RM::SerialMotor::GetInstance(RM::ControlReg::FRONT_BACK_MOTOR);
    };

    // 清晰度检测
    double Clarity(cv::Mat &img) const;
    void MotorAutoRun(cv::Mat &img);
    ~ClarityLocator() {};

  public:
    virtual std::string GetName() const override;
    virtual bool Execute(cv::Mat &img) override;

  public:
    static bool clarity_alert;
    // 1600 的位置是最清晰的
    static int INIT_CLARITY_POS;

  private:
    // 设置locator的名称
    std::string name_;
    // 清晰位置大概在 [1400, 1800] 之间，在之外的都需要通过走马达调整
    const int FRONT_OUT_OF_RANGE_CLARITY_ = 600;
    const int BACK_OUT_OF_RANGE_CLARITY_ = 600;
    const int MIX_STEPS_ = 50;
    const double CLARITY_DEVIATION_ = 0.01;
    const double POS_DEVIATION_ = 5;
    // TODO: set clarity shreshold, if less than it when mannual move front/back
    // motor, it should alert with msg to UI.
    const double CLARITY_SHRD_ = 3.0;
    static std::pair<int, double> max_clear_pos_;
    RM::SerialMotor *front_back_motor_;
};

class Locator : public ImageOperations {
  public:
    Locator(std::string s, RM::CameraRefreshThread *thd = nullptr)
        : name_(s), camera_thread_(thd) {
        left_right_motor_ =
            RM::SerialMotor::GetInstance(RM::ControlReg::LEFT_RIGHT_MOTOR);
        up_down_motor_ =
            RM::SerialMotor::GetInstance(RM::ControlReg::UP_DOWN_MOTOR);

        // put detector initial code here to read templates only once.
        ids.push_back("circle");
        detector.readClasses(ids, "circle_templ.yaml");
    };
  public:
    virtual std::string GetName() const override;
    virtual bool Execute(cv::Mat &img) override;

  public:
    // 8个点检测
    cv::Point MatchDetect(cv::Mat &img);
    cv::Point MatchDetectCircle(cv::Mat &img);
    void MotorStop();
    bool IsFindEye();
    cv::Point FindMatchDetectPoint();
    int GetCircleRadius();
    double GetMatchSimilarity();
    // 马达自动根据8个点位置进行走动
    void MotorAutoRun(const cv::Mat &img, const cv::Point &pt, float k_lr,
                      float k_u, float k_d, int motor_delays);
    void MotorAutoRunCore(cv::Point pt, int fx, int fy, double threshold,
                          double deviation, float k_lr, float k_u, float k_d,
                          int motor_delay);
    ~Locator() {};

  public:
    static cv::Mat template_;
    static cv::Point centerPoint;
    static int circle_radius;
    static double similarity;

  private:
    // 设置locator的名称
    std::string name_;
    std::mutex match_detect_mutex_;
    std::mutex switch_mutex_;
    static std::atomic<bool> findEye_;
    //设置使用的匹配模板
    static cv::Mat result_;
    static cv::Mat read_template();
    // 设置 shape_based_detector 相关
    // feature numbers(how many ori in one templates?)
    // two pyramids, lower pyramid(more pixels) in stride 4, lower in stride 8
    line2Dup::Detector detector{150/*num_feature*/, {4, 8}};
    std::vector<std::string> ids;

    RM::SerialMotor *left_right_motor_;
    RM::SerialMotor *up_down_motor_;
    // 设置刷新线程
    RM::CameraRefreshThread *camera_thread_{nullptr};
};

// 按下按钮需要做的事，示例
class ButtonOps : public ImageOperations {
  public:
    enum Btn_t {
        kTouchBtn,         // 视频区域触摸按键
        kLeftBtn,          // 左右切换马达按键
        kRightBtn,         // 左右切换马达按键
        kFrontBtn,         // 前后切换马达按键
        kBackBtn,          // 前后切换马达按键
        kForeheadUpBtn,    // 额托上下切换马达按键
        kForeheadDownBtn,  // 额托上下切换马达按键
        kStartBtn,
        kAutoMannualBtn  // 自动与手动切换按键
    };

    enum {
      kBeginOpsID = 1<<30,
    };
  
  public:
    explicit ButtonOps() {
      ops_id_ = global_ops_id++;
    }

  public:
    int GetOpsID(void) const {
      return ops_id_;
    }

  public:
    virtual std::string GetName() const override;
    virtual bool Execute(cv::Mat &img) override;
    void SetThread(RM::CameraRefreshThread *thd) {
        thd_ = thd;
    };
    RM::CameraRefreshThread *GetThread() {
        return thd_;
    };
    int GetBtnType() {
        return btn_;
    }

  public:
    void SetCameraID(const int cameraID) {
        camera_id_ = cameraID;
    }
    static bool hasFindClarityPos(){
        return has_find_clarity_pos;
    }
    bool thd_detect_clarity();
  private:
    std::atomic<bool> run_detect_thd_{false};
    int camera_id_ = RM::kMainCameraId;
    void _thd_detect_eight_points(bool motor_directions);
    void detect_and_stop_motor(bool motor_directions);
    bool detect_eight_points(cv::Mat &img);
    bool detect_circle(cv::Mat &img);
    bool _detect_eight_points_once();

  public:
    // 设置长按键还是短按键
    bool SetBtnStatus(bool status) {
        return is_long_btn_ = status;
    }
    bool SetArgs(bool args) {
        return args_b_ = args;
    }
    cv::Point SetArgs(cv::Point args) {
        return args_pt_ = args;
    }
    void SetBtnType(Btn_t t) {
        btn_ = Btn_t(t);
    }
    template <typename T>
    static T Log(T log) {
        LOG(INFO) << log;
        return log;
    }
  public:
    // FIXME: export.cpp also has???, should remove the other later. 
    const int LEFT_EYE_X = -3884;  // 左右
    const int LEFT_EYE_Y = 337;  // 上下
    const int MIDDLE_POS_X = 0;
    const int RIGHT_EYE_X = 2482;//-2107;  // 左右
    const int RIGHT_EYE_Y = 337;   // 上下
    const int EYE_Z = 2425;// 前后


    static bool has_find_clarity_pos;

  private:
    cv::Point args_pt_;
    bool is_long_btn_;
    bool args_b_;
    unsigned char btn_;
    //设置使用的匹配模板
    static cv::Mat template_;
    static cv::Mat result_;
    // 设置在左右眼切换时，正确检测到8个点的默认阈值，这个值不能过大，
    // 过大了，左眼由于会变形，导致很难检测到，这时就会走过
    // 过小了，左眼就会立即检查到，导致还没有走到正中央，马达就停下来了，
    // 这个对于右眼影响不大，因为右眼不会变形
    // TODO: 对于左眼变形的问题，其实是受光路影响，感觉不在同一个准心度上面，后续看光路如何调整
    const double THRESHOLD_ = 0.4;
    std::vector<std::thread> thd_list_;
    RM::CameraRefreshThread *thd_ = nullptr;

    // const int BACK_OUT_OF_RANGE_CLARITY_ = 1600;
    // const int INIT_CLARITY_POS = 900;
    double curSobelVal = 0.0;
    // 每次清晰度查找时，用于存放 sobel 值的队列
    std::vector<double> sobelQ_;
    int direction = 1;
    // TODO: 根据经验值调整
    int stepWindow = 100;
    int loopCnt = 0;
    // 每次清晰度查找，设置需要进行几轮聚焦才能成功
    int loopTotal = 2;
    // 设置清晰度评价经验值，测试下来大于 10.0 比较合适
    const double CLARITY_THRESHOLD_ = 10;
    // 从机器开机到目前为止，每次清晰度查找成功的值的和
    static double sumClarityVal_;
    // 从机器开机到目前为止已经找到了的清晰位置的总次数
    static long clarityTimes_;
    // 从机器开机到目前为止已经找到了的清晰度平均值
    static double aveClarityVal_;
    static std::atomic<int> global_ops_id;
    int ops_id_ = 0;
};

}  // end namespace RM

#endif  //_LOCATOR_H_
