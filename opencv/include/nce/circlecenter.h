//
// Created by Ji, Jerry on 11/19/17.
//

#ifndef NCE_ALGO_DRAW_LINE_H
#define NCE_ALGO_DRAW_LINE_H

#include <stdio.h>

#include <atomic>
#include <map>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "algo/include/types.h"

namespace RM {
// begin of namespace RM

// rows是图片总共的行数
// cols是图片总共的列数
// angle表示直线的度数，从[0 ~ 360)
// r,c表示起始点的位置
// ret存放所有的点的点坐票
void get_line_points(int rows, int cols, int angle, int r, int c,
                     std::vector<std::pair<int, int>> &ret);

void draw_circle(std::vector<std::pair<int, int>> &vs, int xc, int yc, int r);

// 给定一个圆环图片，然后要找出这个图片很粗的圆环的
// 细化之后的中心的那个圆环
// 为了速度考虑，这里只在需要的时候进行二值化
// 所以传进来的图片只需要是灰度图就可以了
// 函数的使用示例，可以看circlecenter_test.cpp
constexpr int kMaxAngle = 360;
void ImageCircleCenter(cv::Mat &gray_src, matrix_base_t xrec[kMaxAngle],
                       matrix_base_t yrec[kMaxAngle], int bin_thresh_value,
                       int circle_row_center, int circle_col_center);

void ImageCircleCenter(cv::Mat &gray_img,
                       std::pair<int,int> start[kMaxAngle],
                       std::pair<int,int> end[kMaxAngle],
                       int bin_thresh_value,
                       int circle_row_center,
                       int circle_col_center);

// 定义于src/circle_template.cpp
// src/circle_template.cpp这个文件不要去更改
// 这个文件是由doc/trans_circle_template.cpp生成
uint8_t *TemplateImageMemAddr(size_t *len);

// 返回模板图片的灰度图
cv::Mat &GrayTemplateImage();

// 在给定的图片中，根据模板找到最匹配的区域
// 为了简少运算量，这里使用灰度图来进行匹配
// src表示原图片，需要是gray image
// tpl表示模板图片，也是gray image
// 返回值表示匹配到的最佳区域
cv::Rect ImageTemplateMatch(cv::Mat &gray_src, cv::Mat &gray_tpl);

// 给定一个带圆环的图片
// 然后要找出这个图片中椭圆的度数
void MeasureEye(cv::Mat &gray_src,     // 输入图片，灰度图
                int bin_thresh_value,  // 二值化的阈值
                matrix_base_t *fa,     // 长轴
                matrix_base_t *fb,     // 短轴
                matrix_base_t *angle,  // 轴位
                double resize = 1.0    // 1.0表示不压缩
);

// 声明一个抽象类，专门用来查找图片的相似度
// 用途在于：
// 当在0位取得人眼的图片之后，可以依次与11张图片进行比较。然后找到最近的图片
// 然后走到相应的位置上去，然后再走到最清晰的位置上
class ImageSimilar {
   public:
    virtual void Compute(cv::Mat &img, void *arg /*自定义输出结果*/) = 0;
};

// 最近邻法来找到最相似的图片
class ImageSimilarKNN : public ImageSimilar {
   public:
    virtual void Compute(cv::Mat &img, void *arg) override;

    explicit ImageSimilarKNN(std::vector<std::pair<int, cv::Mat>> &vs)
        : image_list_(vs) {}
    explicit ImageSimilarKNN(std::vector<std::pair<int, cv::Mat>> &&vs)
        : image_list_(std::move(vs)) {}

   private:
    std::vector<std::pair<int, cv::Mat>> image_list_;
};

class ImageSum {
    std::vector<std::vector<long long>> sum_;
    int add_cnt_ = 0;
    cv::Mat back_ground_;
    bool has_average_ = false;

   public:
    ImageSum() {}
    void Add(cv::Mat &img);

    cv::Mat AverageImage();
    cv::Mat SubAverage(cv::Mat &src);
    void clear();
};

}  // end of namespace RM

#endif  // NCE_ALGO_DRAW_LINE_H
