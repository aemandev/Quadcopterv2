#include <CUnit/CUnit.h>
#include "quaternion.h"

void test_quaternion_new(void) {
    quaternion_t* q = quaternion_new(1.0, 2.0, 3.0, 4.0);
    CU_ASSERT_EQUAL(q->w, 1.0);
    CU_ASSERT_EQUAL(q->x, 2.0);
    CU_ASSERT_EQUAL(q->y, 3.0);
    CU_ASSERT_EQUAL(q->z, 4.0);
    quaternion_destroy(q);
}

void test_getQ(void) {
    quaternion_t* q = quaternion_new(1.0, 2.0, 3.0, 4.0);
    float* Q = getQ(q);
    CU_ASSERT_EQUAL(Q[0], 1.0);
    CU_ASSERT_EQUAL(Q[1], 2.0);
    CU_ASSERT_EQUAL(Q[2], 3.0);
    CU_ASSERT_EQUAL(Q[3], 4.0);
    free(Q);
    quaternion_destroy(q);
}

void test_quat_conj(void) {
    quaternion_t* q = quaternion_new(1.0, 2.0, 3.0, 4.0);
    quaternion_t* q_conj = quat_conj(q);
    CU_ASSERT_EQUAL(q_conj->w, 1.0);
    CU_ASSERT_EQUAL(q_conj->x, -2.0);
    CU_ASSERT_EQUAL(q_conj->y, -3.0);
    CU_ASSERT_EQUAL(q_conj->z, -4.0);
    quaternion_destroy(q);
    quaternion_destroy(q_conj);
}

int main() {
    CU_initialize_registry();
    CU_pSuite suite = CU_add_suite("quaternion_tests", NULL, NULL);
    CU_add_test(suite, "test_quaternion_new", test_quaternion_new);
    CU_add_test(suite, "test_getQ", test_getQ);
    CU_add_test(suite, "test_quat_conj", test_quat_conj);
    CU_basic_run_tests();
    CU_cleanup_registry();
    return 0;
}