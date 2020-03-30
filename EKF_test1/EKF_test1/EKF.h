
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
	/* ���û���������֮��ľ��� */
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

	/* ���ⲿ����۲� Z ��ֵ, �� imu �� odometry ��ֵ */
	void input_Z( double Vr, double Vl, double w, double theta, double V )
	{
		Z(0) = Vr;
		Z(1) = Vl;
		Z(2) = w;
		Z(3) = theta;
		Z(4) = V;
	}
	/* �õ�״̬ X ��ֵ */
	Eigen::Matrix<double, 5, 1> getX()
	{
		return X;
	}
	void init();
	void prediction(  );
	void update(double Vr, double Vl, double w, double theta, double V);
private:
	double l; // ����������֮��ľ���
	Eigen::Matrix<double, 5, 1> X;// ����һ�� 5 ά��״̬����
	Eigen::Matrix<double, 5, 1> X_previous;// ����һ�� 5 ά��״̬�����Ĺ���
	Eigen::Matrix<double, 5, 1> X_estimate;// ����һ�� 5 ά��״̬������ǰһʱ�̵Ĺ���
	Eigen::Matrix<double, 5, 1> Z;// ����һ�� 5 ά�Ĺ۲�����
	Eigen::Matrix<double, 5, 1> Z_estimate;// ����һ�� 5 ά�Ĺ۲������Ĺ���
	Eigen::Matrix<double, 5, 1> Z_previous;// ����һ�� 5 ά�Ĺ۲�������ǰһʱ�̵Ĺ���
	double dt;// ����ʱ��delta t
	Eigen::Matrix<double, 5, 5> P;// ����һ�� 5 * 5 ��ϵͳЭ�������
	Eigen::Matrix<double, 5, 5> P_previous; // ����һ�� 5 * 5 ��ϵͳЭ��������ǰһʱ�̵Ĺ���
	Eigen::Matrix<double, 5, 5> P_estimate; // ����һ�� 5 * 5 ��ϵͳЭ�������Ĺ���
	Eigen::Matrix<double, 5, 5> Q; // ����һ�� 5 * 5 ��ϵͳ����Э�������
	Eigen::Matrix<double, 5, 5> R; // ����һ�� 5 * 5 �Ĺ۲�����Э�������
	Eigen::Matrix<double, 5, 5> H; // ����һ�� 5 * 5 �Ĺ۲����
	Eigen::Matrix<double, 5, 5> F; // ����һ�� 5 * 5 ��״̬ת���ſɱȾ���
	
	//double odom_bias1, odom_bias2;
	//double gyroscop_bias;
	//double heading_bias;
	//double accelerometer_bias;
};

void EKF::init()
{
	// ��һ������ʼ����ʼ״̬X(0), Z(0), P0;
	X_previous << 0, 0, 0, 0, 0;// ��ʼʱ״̬ X Ϊ 0 ��
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
	// �ڶ�����״̬Ԥ��
	X_estimate(0) = X_previous(0) + X_previous(3) * dt * cos(X_previous(2));
	X_estimate(1) = X_previous(1) + X_previous(3) * dt * sin(X_previous(2));
	X_estimate(2) = X_previous(2) + X_previous(4) * dt;
	X_estimate(3) = X_previous(3);
	X_estimate(4) = X_previous(4);

	// ���������۲�Ԥ��
	Z_estimate = H * X_estimate;

	// ���Ĳ���һ�����Ի�״̬���̣�����״̬ת�ƾ��� F
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

	// ���岽�����״̬Э��������Ԥ��
	P_estimate = F * P_previous * F.transpose() + Q;
}

void EKF::update(double Vr, double Vl, double w, double theta, double V)
{
	// ���������� Kalman ����
	Eigen::Matrix<double, 5, 5> temp;
	temp = H * P_estimate.inverse() * H.transpose() + R;
	Eigen::Matrix<double, 5, 5> K = P_estimate.inverse() * H.transpose() * temp.inverse();

	// ����۲�ֵ
	input_Z( Vr, Vl, w, theta, V );

	// ���߲���״̬����
	X = X_estimate + K * ( Z - Z_estimate );
	 
	// �ڰ˲���״̬Э�������
	Eigen::Matrix<double, 5, 5> I = Eigen::MatrixXd::Identity(5, 5);
	P = ( I - K * H ) * P_estimate;

	// 
	P_previous = P;
	X_previous = X;
}

#endif // !__EKF_H_

