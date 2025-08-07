#pragma once

#include <iostream>
#include <cmath>
#include <chrono>
#include <Eigen/Dense>

using namespace Eigen;
using std::cout;
using std::endl;

// 踝关节限幅（位置）
const double MAX_PITCH_DEG = 35.0f;
const double MIN_PITCH_DEG = -35.0f;
const double MAX_ROLL_DEG  = 10.0f;
const double MIN_ROLL_DEG  = -10.0f;
// 踝关节限幅（速度）
const double MAX_PITCH_DEG_DOT = 10.0f;
const double MIN_PITCH_DEG_DOT = -10.0f;
const double MAX_ROLL_DEG_DOT  = 10.0f;
const double MIN_ROLL_DEG_DOT  = -10.0f;


#define LEFT_LEG_A_INDEX    4
#define LEFT_LEG_B_INDEX    5
#define RIGHT_LEG_A_INDEX   10
#define RIGHT_LEG_B_INDEX   11

#define L_BAR               70
#define L_ROD_LONG          250
#define L_ROD_SHORT         185
#define L_SPACING           35

namespace qi {
class ParallelAnkle {
public:
    ParallelAnkle(double l_bar, double l_rod_long, double l_rod_short, double l_spacing);

    // 逆运动学: 输入 roll,pitch (deg)，返回 h1,h2 (deg)
    std::pair<double,double> inverse_kinematics(double roll_deg, double pitch_deg);

    // 前向运动学: 输入 h1,h2 (deg)，迭代求 roll,pitch (deg)
    std::pair<double,double> forward_kinematics(double h1_deg, double h2_deg,
                                                double tol = 1e-6, int max_iter = 10);

    // 雅可比矩阵 2x6
    Matrix<double,2,6> jacobian(double roll_deg, double pitch_deg);

    // 约束雅可比 2x2
    Matrix2d constrained_jacobian(double roll_deg, double pitch_deg);

     /**
     * 关节速度计算: 根据电机角速度 (deg/s) 计算踝关节角速度 (deg/s)
     * h_dot = [h1_dot, h2_dot]
     * q_dot = [q_roll_dot, q_pitch_dot]
     */
    Vector2d computeJointVelocity(double h1_dot_deg, double h2_dot_deg,
                                  double roll_deg, double pitch_deg);
    /**
    * 根据当前的踝关节角度 (deg) 计算踝关节角速度 (deg/s)
    */
    Vector2d computeJointVelocity(double roll_deg, double pitch_deg);
    /**
     * 足端角速度计算: 根据踝关节角速度 (deg/s) 计算电机角速度 (deg/s)
     * q_dot = [q_roll_dot, q_pitch_dot]
     * return: h_dot = [h1_dot, h2_dot]
     */
    Vector2d computeMotorVelocity(double q_roll_dot_deg, double q_pitch_dot_deg,
                                  double roll_deg, double pitch_deg);

    /**
     * 力矩映射: 根据足端力/力矩 (F_roll, F_pitch) 计算电机力矩 (tau1, tau2)
     * tau = Jc^T * F_c
     */
    Vector2d computeMotorTorque(double F_roll, double F_pitch,
                                 double roll_deg, double pitch_deg);
    
    Vector2d computeJointTorque(double F_h1, double F_h2,
                                 double h1_deg, double h2_deg);


private:
    double deg2rad(double d) { return d * M_PI / 180.0; }
    double rad2deg(double r) { return r * 180.0 / M_PI; }

    double solve_motor(const Vector3d& rA, const Vector3d& rC, double lrod);

    Vector3d compute_B(const Vector3d& rA, const Vector3d& rC, double lrod, const Vector3d& prev_B);

private:
    double l_bar, l_rod_long, l_rod_short, l_spacing;
    Vector3d A1, A2, C1, C2;
    Vector3d B1_prev = Vector3d::Zero(), B2_prev = Vector3d::Zero();
    double delta_t = 0.0;            // 解算周期
    Vector2d ankle_vel_prev;     // 脚踝上一时刻的速度
    double t_prev = 0.0;             // 上一时刻系统时间
    double pitch_rad_prev = 0.0;         // 上一时刻pitch角
    double roll_rad_prev = 0.0;          // 上一时刻roll角
};

}
