
#ifndef __EKF_H_
#define __EKF_H_

#include<Eigen\Dense>
#include <iostream>
#include <math.h>

#define M_PI       3.14159265358979323846


class EKF {
public:	
	EKF()
	{

	}
	~EKF()
	{

	}
	/* 设置机器人两轮之间的距离 */
	void setL( double l_ )
	{
		l = l_;
	}
	void setDelta_t( double dt_ )
	{
		dt = dt_;
	}
	void setNoise_R( double odom_bias1, double odom_bias2, \
					 double gyroscop_bias, \
					 double heading_bias, \
					 double accelerometer_bias )
	{
		R = Eigen::MatrixXd::Zero(5, 5);
		R(0, 0) = odom_bias1;
		R(1, 1) = odom_bias2;
		R(2, 2) = gyroscop_bias;
		R(3, 3) = heading_bias;
		R(4, 4) = accelerometer_bias;
	}
	void setNoise_Q(double q0, double q1, double q2, double q3, double q4)
	{
		Q = Eigen::MatrixXd::Zero(5, 5);
		Q(0, 0) = q0;
		Q(1, 1) = q1;
		Q(2, 2) = q2;
		Q(3, 3) = q3;
		Q(4, 4) = q4;
	}

	/* 从外部输入观测 Z 的值, 即 imu 和 odometry 的值 */
	void input_Z( double Vr, double Vl, double w, double theta, double V )
	{
		Z(0) = Vr;
		Z(1) = Vl;
		Z(2) = w;
		Z(3) = theta;
		Z(4) = V;
	}
	/* 得到状态 X 的值 */
	Eigen::Matrix<double, 5, 1> getX()
	{
		return X;
	}
	void init();
	void prediction(  );
	void update(double Vr, double Vl, double w, double theta, double V);
private:
	double l; // 机器人两轮之间的距离
	Eigen::Matrix<double, 5, 1> X;// 定义一个 5 维的状态向量
	Eigen::Matrix<double, 5, 1> X_previous;// 定义一个 5 维的状态向量的估计
	Eigen::Matrix<double, 5, 1> X_estimate;// 定义一个 5 维的状态向量的前一时刻的估计
	Eigen::Matrix<double, 5, 1> Z;// 定义一个 5 维的观测向量
	Eigen::Matrix<double, 5, 1> Z_estimate;// 定义一个 5 维的观测向量的估计
	Eigen::Matrix<double, 5, 1> Z_previous;// 定义一个 5 维的观测向量的前一时刻的估计
	double dt;// 采样时间delta t
	Eigen::Matrix<double, 5, 5> P;// 定义一个 5 * 5 的系统协方差矩阵
	Eigen::Matrix<double, 5, 5> P_previous; // 定义一个 5 * 5 的系统协方差矩阵的前一时刻的估计
	Eigen::Matrix<double, 5, 5> P_estimate; // 定义一个 5 * 5 的系统协方差矩阵的估计
	Eigen::Matrix<double, 5, 5> Q; // 定义一个 5 * 5 的系统噪声协方差矩阵
	Eigen::Matrix<double, 5, 5> R; // 定义一个 5 * 5 的观测噪声协方差矩阵
	Eigen::Matrix<double, 5, 5> H; // 定义一个 5 * 5 的观测矩阵
	Eigen::Matrix<double, 5, 5> F; // 定义一个 5 * 5 的状态转移雅可比矩阵
	
	//double odom_bias1, odom_bias2;
	//double gyroscop_bias;
	//double heading_bias;
	//double accelerometer_bias;
};

void EKF::init()
{
	// 第一步：初始化初始状态X(0), Z(0), P0;
	X_previous << 0, 0, 0, 0, 0;// 初始时状态 X 为 0 ；
	Z << 0, 0, 0, 0, 0;
	P_previous = Eigen::MatrixXd::Identity(5, 5);
	H << 0, 0, 0, 1, 0.5 * l,
		 0, 0, 0, 1, -0.5 * l,
		 0, 0, 0, 0, 1,
		 0, 0, 1, 0, 0,
		 0, 0, 0, 1, 0;
}
void EKF::prediction(  )
{
	// 第二步：状态预测
	X_estimate(0) = X_previous(0) + X_previous(3) * dt * cos(X_previous(2));
	X_estimate(1) = X_previous(1) + X_previous(3) * dt * sin(X_previous(2));
	X_estimate(2) = X_previous(2) + X_previous(4) * dt;
	X_estimate(3) = X_previous(3);
	X_estimate(4) = X_previous(4);

	// 第三步：观测预测
	Z_estimate = H * X_estimate;

	// 第四步：一阶线性化状态方程，求解出状态转移矩阵 F
	F(0, 0) = 1;
	F(0, 1) = 0;
	F(0, 2) = -dt * X_estimate(3) * sin(X_estimate(2));
	F(0, 3) = dt * cos(X_estimate(2));
	F(0, 4) = 0;
	F(1, 0) = 0;
	F(1, 1) = 1; 
	F(1, 2) = dt * X_estimate(3) * cos(X_estimate(2));
	F(1, 3) = dt * sin(X_estimate(2));
	F(1, 4) = 0;
	F(2, 0) = 0;
	F(2, 1) = 0;
	F(2, 2) = 1;
	F(2, 3) = 0;
	F(2, 4) = dt;
	F(3, 0) = 0;
	F(3, 1) = 0;
	F(3, 2) = 0;
	F(3, 3) = 1;
	F(3, 4) = 0;
	F(4, 0) = 0;
	F(4, 1) = 0;
	F(4, 2) = 0;
	F(4, 3) = 0;
	F(4, 4) = 1;

	// 第五步：求解状态协方差矩阵的预测
	P_estimate = F * P_previous * F.transpose() + Q;
}

void EKF::update(double Vr, double Vl, double w, double theta, double V)
{
	// 第六步：求 Kalman 增益
	Eigen::Matrix<double, 5, 5> temp;
	temp = H * P_estimate.inverse() * H.transpose() + R;
	Eigen::Matrix<double, 5, 5> K = P_estimate.inverse() * H.transpose() * temp.inverse();

	// 输入观测值
	input_Z( Vr, Vl, w, theta, V );

	// 第七步：状态更新
	X = X_estimate + K * ( Z - Z_estimate );
	 
	// 第八步：状态协方差更新
	Eigen::Matrix<double, 5, 5> I = Eigen::MatrixXd::Identity(5, 5);
	P = ( I - K * H ) * P_estimate;

	// 
	P_previous = P;
	X_previous = X;
}

#endif // !__EKF_H_

