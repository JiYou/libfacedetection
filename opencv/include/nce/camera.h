#ifndef _NCE_CAMERA_H_
#define _NCE_CAMERA_H_

#ifdef WIN32
#include <Windows.h>
#endif

#include "nce/util.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <atomic>
#include <mutex>
#include <unordered_map>
#include <vector>
#include <condition_variable>

namespace RM {
// namespace RM
// define eight-points camera as main camera, and it's value is 1
const int kMainCameraId = 1;
// 摄像头纯虚类
class Camera {
  public:
    class Frame {
      public:
        virtual double Clearity() const = 0;
        virtual ~Frame() {}
    };

  public:
    virtual bool GetFrame(Frame *f) = 0;
    virtual bool IsOk() const = 0;
    virtual int Width() const = 0;
    virtual int Height() const = 0;
    virtual int GetCameraNumber() const = 0;
    // 把摄像头设置成为高清摄像头，在这种情况下
    // 取得的图片就是高清图片
    // 注意这里设置成功之后，然后要设置回去
    // 保证只在拍照模式下取高清图片
    // 在一般的视频模式下依然使用640 * 480
    // 的模式
    virtual bool GetHighDefinitionImage(Frame *f) = 0;
};

// 基于OpenCV实现的Camera
class OpenCVCamera : public Camera {
  public:
    class OpenCVFrame : public Frame {
        friend class OpenCVCamera;
        cv::Mat frame_;

        // 在等待马达走动的时候依然需要刷新屏幕

      public:
        const cv::Mat &GetData() const {
            return frame_;
        }
        cv::Mat &GetData() {
            return frame_;
        }
        virtual double Clearity() const override;
#ifdef WIN32
        int Bpp() {
            IplImage img = GetData();
            IplImage *m_img = &img;
            return m_img ? (m_img->depth & 255) * m_img->nChannels : 0;
        };
        void FillBitmapInfo(BITMAPINFO *bmi, int width, int height, int bpp,
                            int origin);
        void Show(HDC dc, cv::Mat &img, int x, int y, int w, int h, int from_x,
                  int from_y);
        void DrawToHDC(HDC hDCDst, RECT *pDstRect, cv::Mat &mat);
#endif
      private:
#ifdef WIN32
        unsigned char buffer_[sizeof(BITMAPINFOHEADER) + 1024];
#endif
    };

  public:
    friend class StaticMapperUtil<cv::VideoCapture *, OpenCVCamera>;
    static OpenCVCamera *GetInstance(int cameraID);
    virtual bool GetFrame(Camera::Frame *f) override; /*里面已经带锁*/
    virtual bool IsOk() const override;
    virtual int Width() const override;
    virtual int Height() const override;
    virtual int GetCameraNumber() const override;
    // 利用opencv的设置，将摄像头拍照设置成为高清的模式
    // 取得图片之后，还会把清晰度设置回去
    virtual bool GetHighDefinitionImage(Camera::Frame *f) override;

  private:
    OpenCVCamera(cv::VideoCapture *cap);
    ~OpenCVCamera();

  private:
    cv::VideoCapture &capture_;
    // 在从摄像头读帧的时候，如果是多线程，那么需要上锁
    std::mutex lock_;

    // RAII class
    // 这里主要是用来设置摄像头的高清模式
    // 类析构的时候，自动回退为以前的模式
    class settings { /*NO LOCK*/
      private:
        cv::VideoCapture &cap_;
        double width_;
        double height_;
        double fps_;
        double frc_;
        bool has_set_{false};

      public:
        settings(cv::VideoCapture &cap);
        ~settings();
    };

    settings* high_settings_{ nullptr };

  private:
    bool get_frame(Camera::Frame *f);
};

// 在显示图像的时候，有时候，可能会需要进行各种操作。
// 因此，在CameraRefreshThread可以添加一系列各种针对
// 图像的操作，操作完成之后再继续显示到窗口上
class ImageOperations {
  public:
    // 获取操作的名字
    virtual std::string GetName() const = 0;
    // 进行具体的操作
    // 执行成功返回true, 否则返回false.
    virtual bool Execute(cv::Mat &img) = 0;

  public:
    // 工厂方法，用来创建子类
    // 创建成功返回true, 否则返回false;
    static bool Create(int option, ImageOperations **ops);

  public:
    virtual void SetExecuted() {
        std::unique_lock<std::mutex> l(lock_);
        LOG(INFO) << "ops " << GetName() << " over, signal" << std::endl;
        exec_ = true;
        cond_.notify_all();
    }
    virtual void WaitForExecuted() {
        std::unique_lock<std::mutex> l(lock_);
        LOG(INFO) << "ops " << GetName() << " begin to wait it over"
                  << std::endl;
        cond_.wait(l, [&]() {
            return exec_ == true;
        });
        LOG(INFO) << "ops " << GetName() << " wait over" << std::endl;
    }
    virtual bool IsExecuted() {
        std::unique_lock<std::mutex> l(lock_);
        return exec_;
    }

  private:
    // 是否被执行
    bool exec_ = false;
    std::mutex lock_;
    std::condition_variable cond_;

  public:
    // 设置正确检测到8个点的默认阈值，这个值过小，定位的时候就容易抖动
    const double THRESHOLD_ = 0.5;
    // 提供给 is_find_eye 的阈值, 提供这个值的目的，是为了更好的让前端检测出眼睛
    const double EYE_MATCHED_THRESHOLD_ = 0.35;
    // 设置离屏幕中心位置偏差
    const double DEVIATION_ = 8;
    // 根据经验设置，0 为最快
    const int motor_default_delay_time = 0x07;
    float ratio = 1;
    // UI给出的视频所对应屏幕的中心位置
    // the screen size is 640*480, so the center point should be (320,240).
    // but ui provide us with 900*674, so the center point set to (450,337)
    // here.
    const int screen_center_x = 450;
    const int screen_center_y = 337;
    //cv::Point screen_center{450, 337};
    const int image_center_x = 320;  // img.cols >> 1;
    const int image_center_y = 240;  // img.rows >> 1;

    // 定义马达步数与触摸屏按下时坐标位置的比例关系参数 k
    // 即根据这个 k， 当知道坐标相差多少时，马达要走的真实步数就可以推断出来
    // 根据经验值所得，左右马达向左或向右走动相同的步数，走的距离能保持相同。
    // 但上下马达，当走动的相同的步数时，向下走动的距离要比向上走动的距离要多。
    // 所以，要分别设置这个比例参数 k
    // left_right_motor coefficient
    const float k_lr = 2;
    // motor up coefficient
    const float k_u = 2.5;
    // motor down coefficient
    const float k_d = 2;
    
    // 清晰度的 sobel 算子清晰度阈值
    const float DEFINITION_THRESHOLD = 7;
  public:
    // 左右马达
    // 马达移动的步数范围经验值在[-3600, 4420]，
    // 即向右移动时，当前步数不能超过-3600,向左移动时不能超过 4420
    const int RIGHT_OUT_OF_RANGE_STEPS_ = 700;//-3600;
    const int LEFT_OUT_OF_RANGE_STEPS_ = 6400;//4600;
    const int ALLOW_DIFF_STEPS_ = 6000;

    // 前后马达
    const int FRONT_OUT_OF_RANGE_STEPS_ = 2500;//1300;
    const int BACK_OUT_OF_RANGE_STEPS_ = 0;//900;
    const int FB_ALLOW_DIFF_STEPS_ = 400;
    const int FB_BEST_POS = 1200;

    // 上下马达
    const int UP_OUT_OF_RANGE_STEPS_ = 1000;
    const int DOWN_OUT_OF_RANGE_STEPS_ = -2500;
    const int UD_BEST_POS = 600;
};

/**
 * 类的功能
 *
 *  这个类主要是提供图片给显示图片的人,比如CameraRefreshThread
 *  CameraRefreshThread需要在窗口上不停地画画
 *  但是有时候，我们需要控制显示的画面
 *
 *  因此，ImageShowUpProvider就主要是提供这么一个图片的来源
 *
 * 图片的来源主要有:
 *  - 摄像头
 *  - 需要显示的圆环的图片，左眼的和右眼的
 * 
 * 我们在测量的时候，有时候是需要把图片显示在前端界面上
 */
class ImageShowUpProvider {
  public:
    enum {
        // 注意，这个变量并不表示说是从0号摄像头拿东西!
        kCameraSource = 0,           /* 缺省情况下，显示图片都来自摄像头 */
        kMeasureLeftEyeImage = 1,    /* 要显示的图片来自测量__左__眼的图片 */
        kMeasureRightEyeImage = 2,   /* 要显示的图片来自测量__右__眼的图片 */
        kMeasureJustEyeImage = 3,    /* 要显示的图片来自__刚__测量的图片 */

        kMaxSource = 4,
    };

  private:
    bool get_camera_image(RM::OpenCVCamera *camera,
                          RM::OpenCVCamera::OpenCVFrame *img);

    bool get_measured_left_eye_image(cv::Mat &img, int idx);
    bool get_measured_right_eye_image(cv::Mat &img, int idx);
    bool get_measured_just_eye_image(cv::Mat &img, int idx);

  public:
    bool GetImage(RM::OpenCVCamera* camera,
        RM::OpenCVCamera::OpenCVFrame* img);

    bool SetImageSource(int source, int idx);

    int GetImageSource(void) const;

  private:
    std::atomic<int> image_source_{kCameraSource};
};

// 专门用于刷新前台界面的thread
class CameraRefreshThread {
  public:
    CameraRefreshThread(int width, int height, int hDC, int cameraID);
    ~CameraRefreshThread();

  public:
    int GetCameraNumber() const {
        return camera_id_;
    }

  public:
    void Start();
    void Stop();
    void WaitExit();
    bool AppendOps(ImageOperations *ops);
    bool RemoveOps(const std::string &name);
    void StopOps(std::string who);
    void StartOps(std::string who);

    // 取一幅图片，并且显示在窗口上
    void RefreshWindowOnce();

  private:
    // 其他地方只能是常量引用
    static std::unordered_map<int /*摄像头ID*/, RM::CameraRefreshThread *>
    thread_map_;

  public:
    static const std::unordered_map<int, RM::CameraRefreshThread *>
    &GetThreadMap() {
        return thread_map_;
    }

    bool IsStop() const {
        return stop_;
    }
  
  public:
    bool SetImageSource(int image_source, int idx);

  private:
    void run();
    std::atomic<bool> stop_{true};
    // 退出设置
    std::atomic<bool> exit_{true};
    std::mutex exit_mutex_;
    std::condition_variable exit_cond_;
    // 用来显示的时候的高宽
    // 主要是用来显示
    int width_ = 0;
    int height_ = 0;
    int hDC_ = 0;
    int camera_id_ = RM::kMainCameraId;

  private:
    // 每个图像的hook函数
    // 在显示图像之前需要进行的操作
    // 由于这种操作并不会很多，直接用vector来进行处理
    // 某个时候需要暂时把所有的ops都停掉。
    bool stop_ops_ = false;

    // 为了调试方便，这里需要记录谁把ops停掉了
    std::string who_stop_;
    std::string who_start_;

    bool run_ops(cv::Mat &img);
    bool _run_ops(std::vector<ImageOperations *> &ref, cv::Mat &img);
    std::mutex ops_mutex_;
    std::vector<ImageOperations *> ops_;

    // 显示图片的来源
    ImageShowUpProvider image_source_;

  private:
#ifdef WIN32
    RECT rect_;
#endif
};

// 这里添加一系列针对单一图像的简化的操作接口

}  // namespace RM

#endif
