#include <unity.h>
#include "../src/utils.h"

void setUp(void) {}
void tearDown(void) {}

// With a flat angle (0°) the level_adjust term is zero, so the set point
// depends only on the stick position relative to the dead band boundaries.

void test_stick_centered_returns_zero(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, calculateSetPoint(0.0f, 1500));
}

void test_stick_at_upper_dead_band_boundary_returns_zero(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, calculateSetPoint(0.0f, 1508));
}

void test_stick_at_lower_dead_band_boundary_returns_zero(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, calculateSetPoint(0.0f, 1492));
}

void test_stick_just_above_dead_band(void) {
    // channel_pulse=1509 → set_point = (1509-1508) / 3 = 1/3
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f / 3.0f, calculateSetPoint(0.0f, 1509));
}

void test_stick_just_below_dead_band(void) {
    // channel_pulse=1491 → set_point = (1491-1492) / 3 = -1/3
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f / 3.0f, calculateSetPoint(0.0f, 1491));
}

void test_full_stick_forward(void) {
    // channel_pulse=2000 → set_point = (2000-1508) / 3 = 492/3 = 164
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 164.0f, calculateSetPoint(0.0f, 2000));
}

void test_full_stick_backward(void) {
    // channel_pulse=1000 → set_point = (1000-1492) / 3 = -492/3 = -164
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -164.0f, calculateSetPoint(0.0f, 1000));
}

// The level_adjust term (-angle*15/3 = -angle*5) corrects the set point
// to push the drone back toward level.

void test_positive_angle_reduces_set_point_in_dead_band(void) {
    // angle=10° → level_adjust=150, set_point = (0 - 150) / 3 = -50
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -50.0f, calculateSetPoint(10.0f, 1500));
}

void test_negative_angle_increases_set_point_in_dead_band(void) {
    // angle=-10° → level_adjust=-150, set_point = (0 - (-150)) / 3 = 50
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 50.0f, calculateSetPoint(-10.0f, 1500));
}

void test_angle_and_stick_combined(void) {
    // channel_pulse=1600 → raw = 1600-1508 = 92; angle=6° → level_adjust=90
    // set_point = (92 - 90) / 3 = 2/3
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f / 3.0f, calculateSetPoint(6.0f, 1600));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_stick_centered_returns_zero);
    RUN_TEST(test_stick_at_upper_dead_band_boundary_returns_zero);
    RUN_TEST(test_stick_at_lower_dead_band_boundary_returns_zero);
    RUN_TEST(test_stick_just_above_dead_band);
    RUN_TEST(test_stick_just_below_dead_band);
    RUN_TEST(test_full_stick_forward);
    RUN_TEST(test_full_stick_backward);
    RUN_TEST(test_positive_angle_reduces_set_point_in_dead_band);
    RUN_TEST(test_negative_angle_increases_set_point_in_dead_band);
    RUN_TEST(test_angle_and_stick_combined);
    UNITY_END();
    return 0;
}
