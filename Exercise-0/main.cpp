#include <iostream>
#include "util.h"
#include "algo.h"

#include <Eigen/Dense>

int main(int argc, char const *argv[])
{
    std::cout << "Welcome to the 3D Scanning & Motion Capture exercise!" << std::endl;

    Student student_a{"A", 4, 1};
    std::cout << student_a.info() << std::endl;

    if (student_a.gets_bonus()) 
    {
        std::cout << "Student " << student_a.name << " gets the bonus!" << std::endl;
    }
    else
    {
        std::cout << "Student " << student_a.name << " does NOT get the bonus!" << std::endl;
    }

	std::vector<float> student_grades{4.0f, 1.3f, 2.3f, 3.3f, 1.0f, 1.7f, 2.0f, 2.0f, 2.7f};
	Algos::sort(student_grades);
	std::cout << "Sorted grade list: " << std::endl;

	for(const auto& grade : student_grades)
	{
		std::cout << "- " << grade << std::endl;
	}

    Eigen::Matrix3f A;
    A << -37, 38, 23,
        68, 8, -52,
        -7, -22, 53;

    A *= 1. / 360;

    std::cout << "Inverse matrix:" << std::endl;
    std::cout << A.inverse() << std::endl;

    system("pause");

    return 0;
}
