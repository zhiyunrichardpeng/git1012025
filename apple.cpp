#include <iostream>
#include <eigen3/Eigen/Core>
int main() {
    // Vector definition
    Eigen::Vector3f v(2.0f, 1.0f, 1.0f);

    // Vector output
    // std::cout << v << std::endl;
    // Vector addition
    // std::cout << "Example of add \n";
    // std::cout << v + w << std::endl;
    // // Vector scalar multiplication
    // std::cout << "Example of scalar multiply \n";
    // std::cout << v * 3.0f << std::endl;
    // std::cout << 2.0f * v << std::endl;    


    float angle = 45.0f * M_PI / 180.0f; // 转为弧度
    // Example of matrix
    // std::cout << "Example of matrix \n";
    // Matrix definition
    Eigen::Matrix3f i, j;
    j << cos(angle), -sin(angle), 0,
    sin(angle), cos(angle), 0,
        0, 0, 1;
    i << 1, 0, 1,
        0, 1, 2,
        0, 0, 1;
    // // Matrix output
    // std::cout << "Example of output \n";
    // std::cout << i << std::endl;
    // // Matrix addition i + j
    // std::cout << "Example of matrix addition \n";
    // std::cout << i + j << std::endl;
    // // Matrix scalar multiplication i * 2.0
    // std::cout << "Example of scalar multiplication \n";
    // std::cout << i * 2.0f << std::endl;
    // // Matrix multiplication i * j
    // std::cout << "Example of matrix multiplication \n";
    // std::cout << i * j << std::endl;
    // // Matrix-vector multiplication i * v
    // std::cout << "Example of matrix-vector multiplication \n";
    // std::cout << i * v << std::endl;

    std::cout << "final rsult \n";
    std::cout << i * j * v << std::endl;


    return 0;
}

