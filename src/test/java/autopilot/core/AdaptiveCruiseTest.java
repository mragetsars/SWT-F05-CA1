package autopilot.core;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class AdaptiveCruiseTest {
    private static final double TARGET_SPEED_KMH = 120.0;
    private static final double DELTA = 0.0001;

    @ParameterizedTest
    @CsvSource({
            "0.0, 0.0",
            "0.2, 0.1",
            "5.0, 2.5",
            "80.0, 40.0",
            "120.0, 60.0"
    })
    void givenValidSpeed_whenSafeDistanceFor_thenReturnsHalfTheSpeed(double speedKmh,
                                                                       double expectedDistanceMeters) {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        assertEquals(expectedDistanceMeters, cruise.safeDistanceFor(speedKmh), DELTA);
    }

    @Test
    void givenInvalidSpeed_whenSafeDistanceFor_thenRejectsInput() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        assertAll(
                () -> assertThrows(IllegalArgumentException.class, () -> cruise.safeDistanceFor(-0.1)),
                () -> assertThrows(IllegalArgumentException.class, () -> cruise.safeDistanceFor(Double.NaN)),
                () -> assertThrows(IllegalArgumentException.class,
                        () -> cruise.safeDistanceFor(Double.POSITIVE_INFINITY)),
                () -> assertThrows(IllegalArgumentException.class,
                        () -> cruise.safeDistanceFor(Double.NEGATIVE_INFINITY)));
    }

    @ParameterizedTest
    @CsvSource({
            "5.0, 0.0, 1.0, ACCELERATE, 5.0",
            "5.0, 4.9, 3.0, ACCELERATE, 0.1",
            "120.0, 115.0, 100.0, ACCELERATE, 5.0",
            "120.0, 120.0, 100.0, HOLD, 0.0",
            "120.0, 130.0, 100.0, HOLD, 0.0"
    })
    void givenClearRoad_whenUpdate_thenAcceleratesOnlyUpToTargetOrHolds(
            double targetSpeedKmh,
            double speedKmh,
            double distanceMeters,
            AdaptiveCruise.Mode expectedMode,
            double expectedAmount) {
        AdaptiveCruise cruise = new AdaptiveCruise(targetSpeedKmh);

        AdaptiveCruise.Decision decision = cruise.update(speedKmh, distanceMeters);

        assertDecision(expectedMode, expectedAmount, decision);
    }

    @ParameterizedTest
    @CsvSource({
            "0.0, 0.0, EMERGENCY_BRAKE, 100.0",
            "100.0, 24.999, EMERGENCY_BRAKE, 100.0",
            "100.0, 25.0, EMERGENCY_BRAKE, 100.0",
            "100.0, 25.001, BRAKE, 49.998",
            "100.0, 49.999, BRAKE, 0.002",
            "100.0, 50.0, HOLD, 0.0",
            "100.0, 50.001, ACCELERATE, 5.0"
    })
    void givenDistanceAtSafetyBoundaries_whenUpdate_thenChoosesSafeCruiseMode(
            double speedKmh,
            double distanceMeters,
            AdaptiveCruise.Mode expectedMode,
            double expectedAmount) {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        AdaptiveCruise.Decision decision = cruise.update(speedKmh, distanceMeters);

        assertDecision(expectedMode, expectedAmount, decision);
    }

    @Test
    void givenBrakingVehicle_whenObstacleStaysInsideReleaseDistance_thenKeepsBrakingUntilReleaseBoundary() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        AdaptiveCruise.Decision initialBrake = cruise.update(100.0, 49.0);
        AdaptiveCruise.Decision hysteresisBrake = cruise.update(100.0, 59.999);
        AdaptiveCruise.Decision releasedDecision = cruise.update(100.0, 60.0);

        assertAll(
                () -> assertDecision(AdaptiveCruise.Mode.BRAKE, 2.0, initialBrake),
                () -> assertDecision(AdaptiveCruise.Mode.BRAKE, 10.0, hysteresisBrake),
                () -> assertDecision(AdaptiveCruise.Mode.ACCELERATE, 5.0, releasedDecision));
    }

    @Test
    void givenReleasedBrakingVehicle_whenObstacleReturnsInsideSafeDistance_thenReappliesProportionalBrake() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(100.0, 49.0);
        cruise.update(100.0, 60.0);

        AdaptiveCruise.Decision decision = cruise.update(100.0, 49.0);

        assertDecision(AdaptiveCruise.Mode.BRAKE, 2.0, decision);
    }

    @ParameterizedTest
    @CsvSource({
            "NaN, 1.0",
            "-1.0, 1.0",
            "Infinity, 1.0",
            "1.0, NaN",
            "1.0, -1.0",
            "1.0, Infinity"
    })
    void givenInvalidSpeedOrDistance_whenUpdate_thenRejectsInput(double speedKmh, double distanceMeters) {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        assertThrows(IllegalArgumentException.class, () -> cruise.update(speedKmh, distanceMeters));
    }

    @Test
    void givenBrakingState_whenUpdateRejectsInvalidInput_thenKeepsBrakingStateForNextValidReading() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(100.0, 49.0);

        assertThrows(IllegalArgumentException.class, () -> cruise.update(Double.NaN, 55.0));
        AdaptiveCruise.Decision nextDecision = cruise.update(100.0, 55.0);

        assertDecision(AdaptiveCruise.Mode.BRAKE, 10.0, nextDecision);
    }

    private void assertDecision(AdaptiveCruise.Mode expectedMode,
                                double expectedAmount,
                                AdaptiveCruise.Decision actualDecision) {
        assertAll(
                () -> assertEquals(expectedMode, actualDecision.mode()),
                () -> assertEquals(expectedAmount, actualDecision.amount(), DELTA));
    }
}
