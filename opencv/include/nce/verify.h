#ifndef _RM_VERIFY_H_
#define _RM_VERIFY_H_

#include "nce/camera.h"
#include "nce/irq.h"
#include "nce/motor.h"
#include "nce/serial.h"
#include "nce/util.h"

#include <stdio.h>

#include <atomic>
#include <vector>

namespace RM {
// begin of namespace RM

// 校验操作
// VerifyImageCenter用来帮助校验人员找到图片的中心位置。
class VerifyImageCenter : public ImageOperations {
   public:
    // 获取操作的名字
    virtual std::string GetName() const override;
    // 进行具体的操作
    // 执行成功返回true, 否则返回false.
    virtual bool Execute(cv::Mat &img) override;
};

// 校验操作
// 找到0D眼睛的标准位置
// 这里有两种办法：
// 1. 根据最初找到的长短轴来决定标准位置在什么地方
// 2.
// 复位后，直接一步一步走，然后根据图片的清晰度来决定真正的标准位置在什么地方。
class VerifyFindStdPosition : public ImageOperations {
   public:
    // 获取操作的名字
    virtual std::string GetName() const override;
    // 进行具体的操作
    // 执行成功返回true, 否则返回false.
    virtual bool Execute(cv::Mat &img) override;

   private:
    SerialMotor *light_motor = nullptr;

    // 这里的状态机用来处理复位
    enum {
        kMotorResetNotStart = 1,
        kMotorResetInProgress = 2,
        kMotorResetOver = 3,
    };
    std::atomic<int> reset_motor_{kMotorResetNotStart};

   private:
    enum {
        // 重置后，最大在[-200, +200]
        // 这个范围里面寻找到最清晰的位置
        // 修改的时候，这个数字必须是10的倍数
        kMaxMoveBackSteps = 200,
        // 每个位置需要取3个值
        // 然后求平均，求出最大值
        kMaxValueForEveryPosition = 3,
    };
    // 这里的状态机用来寻找最清晰的位置
    enum {
        // 还没有开始移动
        kMotorMoveBackNotStart = 1,
        // 已经向着-200方向移动了
        KMotorMoveInProgress = 2,
        // 向后面移动结束
        kMotorMoveBackOver = 3,
    };
    std::atomic<int> motor_move_{kMotorMoveBackNotStart};

    struct record {
        // 马达所在位置
        int pos = 0;
        // 这个位置记录的清晰度的值
        std::vector<double> val;
        double get_avg() {
            RM_ASSERT(val.size() > 0);
            double s = 0;
            for (auto &d : val) {
                s += d;
            }
            return s / ((double)val.size());
        }
    };
    std::vector<record> motor_record_;

    int std_pos_for_0d_ = 0;

   public:
    int GetBestPos() { return std_pos_for_0d_; }
};

// 按下按钮之后，这里开始要进行响应
// 这里要做的工作，主要是两方向，
// 1. 一个是找到图片的中心位置 && 调整光路
// 2. 找到0D眼睛，放上去之后，按下开始按钮
void button_action_start_center_and_stdpos_verify();
void button_action_stop_center_and_stdops_verify();

class ImageBuffer {
  public:
    enum {
        kMeasuredLeftEeye = 0,  /* 指向左眼所在的buffer */
        kMeasuredRightEeye = 1, /* 指向右眼图片所在的buffer */
        kLastPushed = 2,        /* 获取最后push的那个buffer */
        kMaxBufferNumber = 3,   /* Buffer的最大数量  */

        kBufferSize = 6,        /* 这里我们最多存放 6 张图片 */
    };

  public:
    // id表示拿到buffer中的第几张
    const cv::Mat& GetImage(int id) const;
    void PutImage(cv::Mat &img);
    int GetImageNumber(void) const;

  private:
    static std::atomic<int> last_put_id_;

  public:
    static ImageBuffer &GetInstance(const int buffer_id);
    static int GetLastId(void);

  private:
    mutable std::mutex lock_;

    // 存放的图片的数量
    int used_{0};

    // 指向可以用的位置
    int iter_{0};

    cv::Mat buffers_[kBufferSize];
};

// 测量的状态机也是在这里实现
class MeasureEyeOps : public ImageOperations {
   public:
    // 注意这里的结果是一个像素值
    struct Result {
        // 长轴
        double fa = 200;

        // 短轴
        double fb = 200;

        // 轴位
        double angle = 0;

        // 这里是以D为单
        // 比如2.5D就是250度
        double degree = 0;
    };

   public:
    struct CircleFeature {
        // 椭圆长轴长
        // 对应libsvm的第1个特征
        double fa = 0;

        // 椭圆短轴长
        // 对应libsvm的第2个特征
        double fb = 0;

        // 偏心率angle
        // 对应libsvm的第3个特征
        double angle = 0;

        // 中径圆的长径的长度
        // 对应libsvm的第4个特征
        double radius = 0;

        // 中径圆的宽度
        // 对应libsvm的第5个特征
        double width = 0;

        std::vector<double> toVector(void) const {
            std::vector<double> ans;
            ans.push_back(fa);
            ans.push_back(fb);
            ans.push_back(angle);
            ans.push_back(radius);
            ans.push_back(width);
            return ans;
        }

        void toVector(std::vector<double> &f) const {
            f.push_back(fa);
            f.push_back(fb);
            f.push_back(angle);
            f.push_back(radius);
            f.push_back(width);
        }
    };

    /*
     * 函数功能：
     *
     * 这里只是打印特征，并不关心具体的特征信息
     * 如果需要说明，那么需要在msg_header上加上说明
     * 这个是什么样的信息
     *
     * 输入参数：
     *      f:              特征信息
     *      label:          样本的标签
     *      msg_header:     说明这个信息的归类
     *
     * 目前使用到msg_header:
     *       F1: 表示用于12个标准眼的分类信息
     *       F2: 只用于没有眼睛和-20D的分类信息
     *
     * NOTE:
     *
     * F1/F2 可能是属于同一种分类信息，也有可能属于不同的分类信息
     *       在区分的时候，需要分开处理
     *
     */
    void PrintFeature(std::vector<double> &f,
                      double label,
                      const std::string msg_header) {
        std::cout << msg_header << " " << label;
        for (int i = 0; i < f.size(); i++) {
            std::cout << " " << i + 1 << ":" << f[i];
        }
        std::cout << std::endl;
    }

    void PrintFeature(std::vector<double> &&f,
                      double label,
                      const std::string msg_header) {
        PrintFeature(f, label, msg_header);
    }

    /*
     * 函数功能：
     *
     * 这里只是打印特征，并不关心具体的特征信息
     * 如果需要说明，那么需要在msg_header上加上说明
     * 这个是什么样的信息
     *
     * 在打印的时候，把信息追加到一个文件中
     *
     * 输入参数：
     *      f:              特征信息
     *      label:          样本的标签
     *      msg_header:     说明这个信息的归类
     *
     * 目前使用到msg_header:
     *       F1: 表示用于12个标准眼的分类信息
     *       F2: 只用于没有眼睛和-20D的分类信息
     *
     * NOTE:
     *
     * F1/F2 可能是属于同一种分类信息，也有可能属于不同的分类信息
     *       在区分的时候，需要分开处理
     *
     */
    void PrintFeatureToFile(std::vector<double> f,
                      double label,
                      const std::string file_name) {
        std::fstream ostr;
        ostr.open(file_name.c_str(),
                   std::fstream::out |
                   std::fstream::binary |
                   std::fstream::app);

        // 输出标签
        ostr << label << " ";

        for (int i = 0; i < f.size(); i++) {
            ostr << " " << i + 1 << ":" << f[i];
        }
        ostr << std::endl;

        ostr.close();
    }

   public:
    /*
     * 函数功能：
     *
     * 提取12个标准眼的特征
     *
     * 输入：
     *      gray:       拿到的原始图的灰度图
     *      frame:      原始彩色图片，主要是为了调试，在上面画图使用
     *      center_row: 返回图片的中心位置 row
     *      center_col: 返回图片的中心位置 col
     *
     * 输出：
     *      CircleFeature 定义好的特征
     */
    CircleFeature GetFeatureAt0D(cv::Mat &frame);
    /*
     * 函数功能：
     *
     * 利用之前训练的结果来进行处理预测。
     * 这里主要是利用12个标准眼来进行预测
     *
     * 输入参数:
     *      f:  利用GetFeatureAt0D() -> CircleFeature 提取的特征来
     *
     * 输出：
     *      kNoEye: 如果发现没有眼睛，输出-100度
     *      label:  如果找到眼睛，输出预测的度数
     */
    double PredictAt0D(CircleFeature &f);

   private:

    // 这里计算图片的长短轴与轴角
    bool compute_length(cv::Mat &gray, int &row_center, int &col_center);

   public:
    // 获取操作的名字
    virtual std::string GetName() const override;

    // 进行具体的操作
    // 执行成功返回true, 否则返回false.
    virtual bool Execute(cv::Mat &img) override;

    // 返回图片处理之后的度数
    double GetDegreeAt0D() const;

    Result GetMeasureResult() const;

    enum {
        // 当返回-100的时候，表示没有眼睛
        kNoEye = -100,

        // 暂且把这两个标准眼的分类放到0D
        k0D90 = 0,
        k0D9180 = 0,
    };

   private:
    double degree_ = 0;
    Result final_result_;

   public:
    MeasureEyeOps();
};

}  // namespace RM

#endif /* ! _RM_VERIFY_H_ */
