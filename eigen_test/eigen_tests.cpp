
#include <iostream>
#include <Eigen/Dense>






int main() {
	// 00
	Eigen::MatrixXd m(2, 2);
	m(0,0) = 1;
	m(1,1) = 1;
	m(1,0) = 0;
	m(0,1) = 0;
	std::cout << m << std::endl;
	// 01
	Eigen::MatrixXd n = Eigen::MatrixXd::Random(3,3);
	n = (n + Eigen::MatrixXd::Constant(3, 3, 1.2)) * 50;
	std::cout << "n =" << std::endl << n << std::endl;
	Eigen::VectorXd v(3);
	v << 1, 2, 3;
	std::cout << "n * v =" << std::endl << n * v << std::endl;
	// 01.1
	Eigen::VectorXd w(3);
	w << 1, 2, 3;
	std::cout << "w * v =" << std::endl << w.array() * v.array() << std::endl;
	
	return 1;
}
