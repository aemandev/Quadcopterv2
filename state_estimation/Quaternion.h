#ifndef QUATERNION_H
#define QUATERNION_H

#include <stdlib.h>

typedef struct {
    float w;
    float x;
    float y;
    float z;
} quaternion_t;

quaternion_t* quaternion_new(float w, float x, float y, float z);
void quaternion_destroy(quaternion_t* q);
float* getQ(quaternion_t* q);
quaternion_t* quat_conj(quaternion_t* q);

#endif /* QUATERNION_H */