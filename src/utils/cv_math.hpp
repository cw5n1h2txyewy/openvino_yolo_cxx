#ifndef CV_MATH_HPP
#define CV_MATH_HPP
#pragma once

#include <opencv2/opencv.hpp>
#include <utility>

namespace detect_utils
{
    constexpr double eps = 1e-12;

    /**
     * @brief 计算两个二维点之间的距离
     * @param p1 第一个点
     * @param p2 第二个点
     * @return 返回两点之间的距离（double类型）
     */
    double calc_point_distance(
        const cv::Point2d &p1,
        const cv::Point2d &p2);

    void test_calc_point_distance();

    enum class ProjectionMode
    {
        InfiniteLine,  // 无限直线
        ClampToSegment // 线段投影到线段上
    };

    /**
     * @brief 计算点到线段的距离
     * @param point 要计算距离的点
     * @param segment_start 线段的起点
     * @param segment_end 线段的终点
     * @param projection_mode 投影模式，有 InfiniteLine 和 ClampToSegment 两种
     * @return 返回点到线段的距离（double类型）
     */
    double calc_point_to_segment_distance(
        const cv::Point2d &point,
        const cv::Point2d &segment_start,
        const cv::Point2d &segment_end,
        const ProjectionMode projection_mode = ProjectionMode::InfiniteLine);

    void test_calc_point_to_segment_distance();

    /**
     * @brief 这个函数计算的是由三个二维点 a、b、c 构成的两个向量 AB 和 AC 的二维叉乘（Cross Product）结果（即叉积在Z轴方向的标量值）。
     * @param a 第一个点
     * @param b 第二个点
     * @param c 第三个点
     * @return 返回叉积在Z轴方向的标量值（double类型）
     */
    double cross(const cv::Point2d &a, const cv::Point2d &b, const cv::Point2d &c);

    /**
     * @brief 判断点 p 是否严格位于由点 ab 上
     * @param a 线段的起点
     * @param b 线段的终点
     * @param p 要判断的点
     * @return 返回点 p 是否在线段 ab 上（bool类型）
     */
    bool on_segment(const cv::Point2d &a, const cv::Point2d &b, const cv::Point2d &p);

    /**
     * @brief 判断两条线段是否相交
     *        核心思想是：如果两条线段相交，那么其中一条线段的两个端点，必然分布在另一条线段所在直线的两侧。
     * @param p1 线段1的起点
     * @param p2 线段1的终点
     * @param q1 线段2的起点
     * @param q2 线段2的终点
     * @return 返回两条线段是否相交（bool类型）
     */
    bool segments_intersect(
        const cv::Point2d &p1,
        const cv::Point2d &p2,
        const cv::Point2d &q1,
        const cv::Point2d &q2);

    void test_segments_intersect();

    enum class CoordSystem
    {
        OpenCV, // OpenCV坐标系
        Math    // Math坐标系
    };

    /**
     * @brief 计算两个点之间的角度
     * @param segment_start 起始点
     * @param segment_end 终止点
     * @param coord_system 坐标系类型（OpenCV或Math）
     * @return 返回角度（度）
     */
    double calc_segment_angle(
        const cv::Point2d &segment_start,
        const cv::Point2d &segment_end,
        const CoordSystem coord_system = CoordSystem::OpenCV);

    void test_calc_segment_angle();

    /**
     * @brief 计算目标点绕原点旋转后的新坐标
     * @param origin_point 旋转中心
     * @param target_point 旋转前的目标点
     * @param rotation_angle 旋转角度（度）；正角度在 OpenCV 坐标系中顺时针旋转，在 Math 坐标系中逆时针旋转
     * @param coord_system 坐标系类型（OpenCV或Math）
     * @return 返回旋转后的目标点坐标
     */
    cv::Point2d calc_rotated_point(
        const cv::Point2d &origin_point,
        const cv::Point2d &target_point,
        double rotation_angle,
        CoordSystem coord_system = CoordSystem::OpenCV);

    void test_calc_rotated_point();

    enum class PerpendicularBasePoint
    {
        StartPoint, // 垂线起点取线段起点
        EndPoint,   // 垂线起点取线段终点
        MidPoint    // 垂线起点取线段中点
    };

    enum class PerpendicularDirection
    {
        // 有向线段（segment_start -> segment_end）的左侧（数学约定，与 cross 函数"左侧为正"保持一致）：
        // 垂线方向向量为 (-dy, dx)，即 cross(segment_start, segment_end, 垂线终点) > 0；
        // 在 Math 坐标系（y 轴向上）中为线段行进方向的左侧。
        Left,
        // 有向线段（segment_start -> segment_end）的右侧：垂线方向向量为 (dy, -dx)，
        // 即 cross(segment_start, segment_end, 垂线终点) < 0；
        // 在 Math 坐标系（y 轴向上）中为线段行进方向的右侧。
        Right
    };

    /**
     * @brief 计算一条线段的垂线
     *        垂线的起点为线段上选定的点（起点、终点或中点），垂线方向为线段的垂直方向（左侧或右侧），长度可指定
     * @param segment_start 线段的起点
     * @param segment_end 线段的终点
     * @param base_point_mode 垂线起点的选取方式：StartPoint（线段起点）、EndPoint（线段终点）或 MidPoint（线段中点）
     * @param direction 垂线方向：Left（左侧，对应垂直向量 (-dy, dx)）或 Right（右侧，对应垂直向量 (dy, -dx)）
     * @param length 垂线长度，默认为 1；传入负数时等价于方向取反，长度为 |length|
     * @return 返回垂线的两个端点（std::pair），first 为垂线起点（即线段上选定的点），second 为垂线终点；
     *         线段退化为一个点时，垂线方向无定义，此时返回的两个端点均为该退化点
     */
    std::pair<cv::Point2d, cv::Point2d> calc_perpendicular_line(
        const cv::Point2d &segment_start,
        const cv::Point2d &segment_end,
        const PerpendicularBasePoint base_point_mode = PerpendicularBasePoint::StartPoint,
        const PerpendicularDirection direction = PerpendicularDirection::Left,
        const double length = 1.0);

    void test_calc_perpendicular_line();
}

#endif // CV_MATH_HPP
