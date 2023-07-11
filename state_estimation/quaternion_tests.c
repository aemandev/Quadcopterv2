#include "Quaternion.h"
#include <stdio.h>
#include <assert.h>

// // Write unit test using normal asserts (not CU_ASSERT_EQUAL)
// void test_quaternion_new(void) {
//     printf("Hello, world!\n");
//     quaternion_t* q = quaternion_new(1.0, 2.0, 3.0, 4.0);
//     // // assert(q->w == 1.0);
//     // // assert(q->x == 2.0);
//     // // assert(q->y == 3.0);
//     // // assert(q->z == 4.0);
//     // quaternion_destroy(q);
// }

int main() {
    printf("Hello, world!\n");
    quaternion_t* q = quaternion_new(1.0, 2.0, 3.0, 4.0);
    printf("Hello, world!\n");
    return 0;
}