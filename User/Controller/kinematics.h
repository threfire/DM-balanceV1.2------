#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <math.h>

#define KIN_EPS 1e-6f
#define KIN_PI  3.141592653589793f

/* 工具末端相对于关节6坐标系的偏移（沿z轴正向） */
#define TOOL_OFFSET_Z 0.18f   // 单位：米
void mat_mul44(const float A[16], const float B[16], float C[16]);
/**
 * 正运动学：计算从基座到指定关节的齐次变换矩阵
 * @param theta  关节角度数组（6个，单位：弧度）
 * @param joint  目标关节编号（0~6，0表示基座，6表示末端）
 * @param T       输出变换矩阵（16个元素，按行主序）
 */
void forward_kinematics(const float theta[6], int joint, float T[16]);

/**
 * 逆运动学：根据末端位姿计算关节角度（多解，选择最接近当前角度的解）
 * @param T_target  目标末端位姿（16个元素，按行主序）—— 这里应为球腕中心位姿
 * @param theta_cur 当前关节角度（6个，单位：弧度）
 * @param theta_out 输出的关节角度（6个，单位：弧度）
 * @return 1 表示求解成功，0 表示无解
 */
int inverse_kinematics_puma(const float T_target[16], const float theta_cur[6], float theta_out[6]);

/**
 * 根据期望的工具末端位姿，计算目标关节角度（自动处理工具偏移）
 * @param T_tool_des  期望的工具末端位姿（16个元素，按行主序）
 * @param theta_cur   当前关节角度（6个，弧度）
 * @param theta_des   输出：目标关节角度（6个，弧度）
 * @return 1 表示成功，0 表示无解
 */
int compute_desired_joint_angles(const float T_tool_des[16], const float theta_cur[6], float theta_des[6]);

/**
 * 根据当前关节角度，计算工具末端位姿（正运动学+工具偏移）
 * @param theta       当前关节角度（6个，弧度）
 * @param T_tool      输出：工具末端位姿（16个元素，按行主序）
 */
void compute_tool_pose(const float theta[6], float T_tool[16]);
/**
 * 工具坐标系下的相对平移
 * @param T_current 当前工具末端位姿（4x4矩阵，行主序）
 * @param dx,dy,dz  在工具坐标系下的平移量（米）
 * @param T_new     输出：新工具末端位姿
 */
void tool_relative_translate(const float T_current[16], float dx, float dy, float dz, float T_new[16]);

/**
 * 工具坐标系下的相对旋转（绕工具坐标系的X、Y、Z轴依次旋转）
 * @param T_current 当前工具末端位姿
 * @param rx,ry,rz  绕工具X、Y、Z轴的旋转角（弧度，按顺序：先绕X，再绕Y，最后绕Z）
 * @param T_new     输出：新工具末端位姿
 */
void tool_relative_rotate_rpy(const float T_current[16], float rx, float ry, float rz, float T_new[16]);

/**
 * 工具坐标系下的通用相对变换（用户提供相对变换矩阵）
 * @param T_current 当前工具末端位姿
 * @param T_rel     相对变换矩阵（在工具坐标系下描述的变换）
 * @param T_new     输出：新工具末端位姿
 */
void tool_relative_transform(const float T_current[16], const float T_rel[16], float T_new[16]);

/**
 * 生成平移变换矩阵（相对于当前坐标系）
 * @param dx,dy,dz  平移量（米）
 * @param T_rel     输出：平移变换矩阵（4x4）
 */
void make_translation_matrix(float dx, float dy, float dz, float T_rel[16]);

/**
 * 生成绕X轴旋转的变换矩阵
 * @param angle  旋转角（弧度）
 * @param T_rel  输出：旋转变换矩阵
 */
void make_rotation_x(float angle, float T_rel[16]);

/**
 * 生成绕Y轴旋转的变换矩阵
 */
void make_rotation_y(float angle, float T_rel[16]);

/**
 * 生成绕Z轴旋转的变换矩阵
 */
void make_rotation_z(float angle, float T_rel[16]);

//
//轨迹规划
// 在 kinematics.h 末尾添加以下声明

/**
 * 将旋转矩阵转换为四元数 (w, x, y, z)
 * @param R 3x3 矩阵，按行主序存放，长度为9
 * @param q 输出四元数，长度为4
 */
void mat3_to_quat(const float R[9], float q[4]);

/**
 * 四元数球面线性插值 (slerp)
 * @param qa 起始四元数
 * @param qb 目标四元数
 * @param t  插值系数 [0,1]
 * @param q  输出四元数
 */
void quat_slerp(const float qa[4], const float qb[4], float t, float q[4]);

/**
 * 四元数转换为旋转矩阵 (行主序)
 * @param q 四元数 (w, x, y, z)
 * @param R 输出3x3矩阵，长度为9
 */
void quat_to_mat3(const float q[4], float R[9]);

/**
 * 齐次矩阵线性插补（位置线性，姿态四元数slerp）
 * @param T_start 起始位姿矩阵（16元素，行主序）
 * @param T_end   目标位姿矩阵
 * @param t       插值系数 [0,1]
 * @param T_out   输出插值位姿矩阵
 */
void pose_interpolate(const float T_start[16], const float T_end[16], float t, float T_out[16]);
#endif
