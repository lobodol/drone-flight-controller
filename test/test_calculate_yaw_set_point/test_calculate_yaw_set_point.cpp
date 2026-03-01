#include <unity.h>
#include "../src/utils.h"

void setUp(void) {}
void tearDown(void) {}

void test_throttle_idle_returns_zero(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, calculateYawSetPoint(2000, 1050));
}

void test_throttle_below_threshold_returns_zero(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, calculateYawSetPoint(2000, 800));
}

void test_throttle_above_threshold_delegates_to_calculate_set_point(void) {
    // calculateSetPoint(0, yaw_pulse): angle=0 so no level adjust
    // yaw_pulse=2000 → (2000-1508) / 3 = 492/3 = 164
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 164.0f, calculateYawSetPoint(2000, 1051));
}

void test_yaw_stick_centered_with_active_throttle_returns_zero(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, calculateYawSetPoint(1500, 1500));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_throttle_idle_returns_zero);
    RUN_TEST(test_throttle_below_threshold_returns_zero);
    RUN_TEST(test_throttle_above_threshold_delegates_to_calculate_set_point);
    RUN_TEST(test_yaw_stick_centered_with_active_throttle_returns_zero);
    UNITY_END();
    return 0;
}
