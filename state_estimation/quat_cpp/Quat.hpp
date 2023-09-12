#include <iostream>
#include <vector>
class Quat{
    public:
        float w;
        float x;
        float y;
        float z;
        Quat();
        Quat(float w, float x, float y, float z);
        // Overload a vector into a quaternion
        Quat(std::vector<float> v, const float& dt);
        Quat(std::vector<float> v);
        Quat& conjugate();
        Quat& inverse();
        Quat& norm();

        Quat operator*(const Quat& q2) const;
        Quat operator+(const Quat& q2) const;   
        Quat operator-(const Quat& q2) const;

        std::vector<float> rotate_vector(std::vector<float>& v);

        friend std::ostream& operator<<(std::ostream& os, const Quat& quat);


};

