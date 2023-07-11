// Quaternion.hpp
#include <cstdint>

typedef struct __attribute__((packed)) {
    int16_t q_array_wxyz[4];
} quat_data_t;


class Quaternion {
   public:
      float w;
      float x;
      float y;
      float z;

      // Unit quaternion
      Quaternion();
      Quaternion(float w, float x, float y, float z);
      Quaternion(const quat_data_t q_array_wxyz[4]);
      Quaternion(float roll, float pitch, float yaw);
      
      float W() const;
      float X() const;
      float Y() const;
      float Z() const;

      Quaternion(const Quaternion &q);

      void Set(float w, float x, float y, float z);
      //void Set(const float q_array_wxyz[4]);
      void Set(const quat_data_t q_array_wxyz[4]);
      Quaternion normalize() const;
      Quaternion conjugate() const;

      void ToArray(quat_data_t q_array[4]) const;
      Quaternion& invert();
      Quaternion& from_RPY(float roll, float pitch, float yaw);

   private:
      float w_;
      float x_;
      float y_;
      float z_;
};



// class Quaternion
// {
// public:
//   float w;
//   float x;
//   float y;
//   float z;

//   Quaternion();
//   Quaternion(float w_, float x_, float y_, float z_);
//   Quaternion(const Vector& u, const Vector& v);
//   Quaternion(float roll, float pitch, float yaw);

//   Vector rotate(const Vector& v) const;
//   Quaternion& normalize();
//   Quaternion inverse() const;
//   Quaternion& invert();
//   Quaternion& from_two_unit_vectors(const Vector& u, const Vector& v);
//   Quaternion& from_RPY(float roll, float pitch, float yaw);
//   void get_RPY(float* roll, float* pitch, float* yaw) const;

//   Vector operator*(const Vector& v) const;
//   Quaternion operator*(const Quaternion& q) const;
//   Quaternion& operator*=(const Quaternion& q);
//   Vector boxminus(const Quaternion& q) const;
//   static Vector log(const Quaternion& q)
//   {
//     Vector v{q.x, q.y, q.z};
//     float norm_v = v.norm();

//     Vector out;
//     if (norm_v < 1e-8)
//     {
//       out.x = out.y = out.z = 0.0;
//     }
//     else
//     {
//       out = 2.0 * atan2(norm_v, q.w) * v / norm_v;
//     }
//     return out;
//   }

//   Vector operator-(const Quaternion& q) const { return boxminus(q); }
// };