#include <unity.h>
#include "../src/utils.h"

void setUp(void) {}
void tearDown(void) {}

void test_value_within_range_is_returned_unchanged(void) {
    TEST_ASSERT_EQUAL_FLOAT(5.0f, minMax(5.0f, 0.0f, 10.0f));
}

void test_value_above_max_is_clamped_to_max(void) {
    TEST_ASSERT_EQUAL_FLOAT(10.0f, minMax(15.0f, 0.0f, 10.0f));
}

void test_value_below_min_is_clamped_to_min(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, minMax(-5.0f, 0.0f, 10.0f));
}

void test_value_equal_to_max_is_returned_unchanged(void) {
    TEST_ASSERT_EQUAL_FLOAT(10.0f, minMax(10.0f, 0.0f, 10.0f));
}

void test_value_equal_to_min_is_returned_unchanged(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, minMax(0.0f, 0.0f, 10.0f));
}

void test_negative_range(void) {
    TEST_ASSERT_EQUAL_FLOAT(-400.0f, minMax(-500.0f, -400.0f, 400.0f));
    TEST_ASSERT_EQUAL_FLOAT(400.0f, minMax(500.0f, -400.0f, 400.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, minMax(0.0f, -400.0f, 400.0f));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_value_within_range_is_returned_unchanged);
    RUN_TEST(test_value_above_max_is_clamped_to_max);
    RUN_TEST(test_value_below_min_is_clamped_to_min);
    RUN_TEST(test_value_equal_to_max_is_returned_unchanged);
    RUN_TEST(test_value_equal_to_min_is_returned_unchanged);
    RUN_TEST(test_negative_range);
    UNITY_END();
    return 0;
}
