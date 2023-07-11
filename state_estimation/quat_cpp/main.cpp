#include "Quat.hpp"
#include <iostream>

int main(){
    Quat q2(1, 2, 3, 4);
    std::cout << q2 << std::endl;

    Quat &q3 = q2.conjugate();
    std::cout << q3 << std::endl;

    Quat q4 = q2.norm();
    std::cout << q4 << std::endl;

    // print q2
    std::cout << q2 << std::endl;

    //print q3
    std::cout << q3 << std::endl;

    // Test vector constructor
    std::vector<float> v = {1, 2, 3};
    Quat q5(v, 0.01);
    std::cout << q5 << std::endl;
    
    std::vector<float> v2 = {1,2,3};
    Quat q6(v2);
    std::cout << q6 << std::endl;

    // Test multiplication
    Quat q7 = q5 * q6;

    // Test Rotate Vector
    // Initialize a unit quaternion
    Quat q8(.6, .3, .2, .7);
    std::vector<float> v3 = {1, 2, 3};
    std::vector<float> v4 = q8.rotate_vector(v3);
    std::cout << v4[0] << " " << v4[1] << " " << v4[2] << std::endl;

    return 0;
}