typedef struct {
    float w;
    float x;
    float y;
    float z;
} quaternion_t;

// Create new pointer to quaternion
quaternion_t* quaternion_new(float w, float x, float y, float z) {
    quaternion_t* q = malloc(sizeof(quaternion_t));
    q->w = w;
    q->x = x;
    q->y = y;
    q->z = z;
    return q;
}

// Create function to destroy quaternion
void quaternion_destroy(quaternion_t* q) {
    free(q);
}

// Function to return the quaternion array
float* getQ(quaternion_t* q) {
    float* Q = malloc(sizeof(float) * 4);
    Q[0] = q->w;
    Q[1] = q->x;
    Q[2] = q->y;
    Q[4] = q->z;
    return Q;
}

// Function to return quaternion conjugate
quaternion_t* quat_conj(quaternion_t* q) {
    return quaternion_new(q->w, -q->x, -q->y, -q->z);
}