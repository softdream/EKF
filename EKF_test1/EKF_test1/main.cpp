#include<Eigen\Dense>
#include<iostream>
#include "EKF.h"

int main()
{
	EKF k;
	Eigen::Matrix<double, 5, 5> X;// ����һ�� 5 ά��״̬����
	X = Eigen::MatrixXd::Zero(5, 5);
	std::cout << "X : "<<std::endl << X << std::endl;
	return 0;
}
	