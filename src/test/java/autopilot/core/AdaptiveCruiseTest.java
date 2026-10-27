package autopilot.core;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class AdaptiveCruiseTest {
    private static final double TARGET_SPEED_KMH = 120.0;
    private static final double DELTA = 1.0e-9;

    @ParameterizedTest(name = "speed={0} km/h -> safe distance={1} m")
    @CsvSource({"0.0, 0.0", "81.5, 40.75"})
    void givenValidSpeed_whenSafeDistanceFor_thenReturnsHalfTheSpeed(
            double speedKmh, double expectedDistanceMeters) {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        assertEquals(expectedDistanceMeters, cruise.safeDistanceFor(speedKmh), DELTA);
    }

    @ParameterizedTest(name = "Reject invalid speed {0}")
    @ValueSource(doubles = {-0.1, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void givenInvalidSpeed_whenSafeDistanceFor_thenRejectsInput(double speedKmh) {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        assertThrows(IllegalArgumentException.class, () -> cruise.safeDistanceFor(speedKmh));
    }

    @ParameterizedTest(name = "target={0}, speed={1}, distance={2} -> {3} {4}")
    @CsvSource({
            "120.0, 0.0,   1.0,   ACCELERATE, 5.0",
            "120.0, 119.9, 100.0, ACCELERATE, 0.1",
            "120.0, 120.0, 100.0, HOLD, 0.0",
            "120.0, 130.0, 100.0, HOLD, 0.0",
            "0.0,   0.0,   1.0,   HOLD, 0.0"
    })
    void givenClearRoad_whenUpdate_thenAcceleratesOnlyUpToTargetOrHolds(
            double targetSpeedKmh, double speedKmh, double distanceMeters,
            AdaptiveCruise.Mode expectedMode, double expectedAmount) {
        AdaptiveCruise cruise = new AdaptiveCruise(targetSpeedKmh);

        AdaptiveCruise.Decision decision = cruise.update(speedKmh, distanceMeters);

        assertDecision(expectedMode, expectedAmount, decision);
    }

    @ParameterizedTest(name = "speed={0}, distance={1} -> {2} {3}")
    @CsvSource({
            "0.0,   0.0,    EMERGENCY_BRAKE, 100.0",
            "100.0, 24.999, EMERGENCY_BRAKE, 100.0",
            "100.0, 25.0,   EMERGENCY_BRAKE, 100.0",
            "100.0, 25.001, BRAKE, 49.998",
            "100.0, 49.999, BRAKE, 0.002",
            "100.0, 50.0,   HOLD, 0.0",
            "100.0, 50.001, ACCELERATE, 5.0"
    })
    void givenDistanceAtSafetyBoundaries_whenUpdate_thenChoosesSafeCruiseMode(
            double speedKmh, double distanceMeters,
            AdaptiveCruise.Mode expectedMode, double expectedAmount) {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        AdaptiveCruise.Decision decision = cruise.update(speedKmh, distanceMeters);

        assertDecision(expectedMode, expectedAmount, decision);
    }

    @ParameterizedTest(name = "Already braking; distance={0} -> {1} {2}")
    @CsvSource({
            "40.0,   BRAKE, 20.0",
            "50.0,   BRAKE, 10.0",
            "59.999, BRAKE, 10.0",
            "60.0,   ACCELERATE, 5.0"
    })
    void givenBrakingVehicle_whenDistanceChanges_thenUsesProportionalOrHysteresisBrakeUntilRelease(
            double distanceMeters, AdaptiveCruise.Mode expectedMode, double expectedAmount) {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        // خوانش اول فقط سابقهٔ ترمز را می‌سازد؛ تصمیم خوانش دوم موضوع تست است.
        cruise.update(100.0, 49.0);

        AdaptiveCruise.Decision decision = cruise.update(100.0, distanceMeters);

        assertDecision(expectedMode, expectedAmount, decision);
    }

    @Test
    void givenBrakingVehicle_whenDistanceReachesEmergencyBoundary_thenEmergencyOverridesHysteresis() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(100.0, 49.0);

        AdaptiveCruise.Decision decision = cruise.update(100.0, 25.0);

        assertDecision(AdaptiveCruise.Mode.EMERGENCY_BRAKE, 100.0, decision);
    }

    @ParameterizedTest(name = "Already braking; new speed={0}, distance={1} -> {2} {3}")
    @CsvSource({"80.0, 50.0, ACCELERATE, 5.0", "120.0, 65.0, BRAKE, 10.0"})
    void givenBrakingVehicle_whenSpeedChanges_thenRecalculatesSafeAndReleaseDistances(
            double speedKmh, double distanceMeters,
            AdaptiveCruise.Mode expectedMode, double expectedAmount) {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(100.0, 49.0);

        AdaptiveCruise.Decision decision = cruise.update(speedKmh, distanceMeters);

        assertDecision(expectedMode, expectedAmount, decision);
    }

    @Test
    void givenEmergencyBrakingVehicle_whenDistanceReachesSafeBoundary_thenKeepsHysteresisBrake() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(100.0, 25.0);

        AdaptiveCruise.Decision decision = cruise.update(100.0, 50.0);

        assertDecision(AdaptiveCruise.Mode.BRAKE, 10.0, decision);
    }

    @Test
    void givenReleasedBrakingVehicle_whenDistanceReturnsToSafeBoundary_thenHoldsSpeed() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(100.0, 40.0);
        cruise.update(100.0, 60.0);

        AdaptiveCruise.Decision decision = cruise.update(100.0, 50.0);

        assertDecision(AdaptiveCruise.Mode.HOLD, 0.0, decision);
    }

    @ParameterizedTest(name = "Reject speed={0}, distance={1}")
    @CsvSource({
            "-0.1,     55.0",
            "NaN,      55.0",
            "Infinity, 55.0",
            "-Infinity,55.0",
            "100.0, -0.1",
            "100.0, NaN",
            "100.0, Infinity",
            "100.0, -Infinity"
    })
    void givenInvalidSpeedOrDistance_whenUpdate_thenRejectsInput(double speedKmh, double distanceMeters) {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        assertThrows(IllegalArgumentException.class, () -> cruise.update(speedKmh, distanceMeters));
    }

    @ParameterizedTest(name = "Preserve braking after rejected speed={0}, distance={1}")
    @CsvSource({"NaN, 55.0", "100.0, NaN"})
    void givenBrakingState_whenUpdateRejectsInvalidInput_thenKeepsBrakingStateForNextValidReading(
            double speedKmh, double distanceMeters) {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(100.0, 49.0);

        assertThrows(IllegalArgumentException.class, () -> cruise.update(speedKmh, distanceMeters));
        AdaptiveCruise.Decision nextDecision = cruise.update(100.0, 55.0);

        assertDecision(AdaptiveCruise.Mode.BRAKE, 10.0, nextDecision);
    }

    private void assertDecision(AdaptiveCruise.Mode expectedMode,
                                double expectedAmount, AdaptiveCruise.Decision actualDecision) {
        assertAll(
                () -> assertEquals(expectedMode, actualDecision.mode()),
                () -> assertEquals(expectedAmount, actualDecision.amount(), DELTA));
    }
}
