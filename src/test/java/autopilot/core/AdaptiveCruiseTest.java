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
            "5.0, 2.5",
            "120.0, 60.0"
    })
    void givenValidSpeed_whenSafeDistanceFor_thenReturnsHalfTheSpeed(double speedKmh,
                                                                       double expectedDistanceMeters) {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        assertEquals(expectedDistanceMeters, cruise.safeDistanceFor(speedKmh), DELTA);
    }

    @Test
    void givenClearRoadBelowTargetSpeed_whenUpdate_thenAcceleratesByBoundedStep() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        AdaptiveCruise.Decision decision = cruise.update(80.0, 100.0);

        assertDecision(AdaptiveCruise.Mode.ACCELERATE, 5.0, decision);
    }

    @Test
    void givenClearRoadAtTargetSpeed_whenUpdate_thenHoldsSpeed() {
        AdaptiveCruise cruise = new AdaptiveCruise(80.0);

        AdaptiveCruise.Decision decision = cruise.update(80.0, 100.0);

        assertDecision(AdaptiveCruise.Mode.HOLD, 0.0, decision);
    }

    @Test
    void givenDistanceAtSafeBoundary_whenUpdate_thenHoldsSpeed() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        AdaptiveCruise.Decision decision = cruise.update(50.0, 25.0);

        assertDecision(AdaptiveCruise.Mode.HOLD, 0.0, decision);
    }

    @Test
    void givenDistanceBetweenEmergencyAndSafeBoundaries_whenUpdate_thenBrakesProportionally() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        AdaptiveCruise.Decision decision = cruise.update(50.0, 20.0);

        assertDecision(AdaptiveCruise.Mode.BRAKE, 20.0, decision);
    }

    @Test
    void givenDistanceAtEmergencyBoundary_whenUpdate_thenEmergencyBrakes() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        AdaptiveCruise.Decision decision = cruise.update(100.0, 25.0);

        assertDecision(AdaptiveCruise.Mode.EMERGENCY_BRAKE, 100.0, decision);
    }

    @Test
    void givenBrakingVehicle_whenDistanceReachesReleaseBoundary_thenResumesAcceleration() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(100.0, 25.0);

        AdaptiveCruise.Decision hysteresisDecision = cruise.update(100.0, 55.0);
        AdaptiveCruise.Decision releasedDecision = cruise.update(100.0, 60.0);

        assertAll(
                () -> assertDecision(AdaptiveCruise.Mode.BRAKE, 10.0, hysteresisDecision),
                () -> assertDecision(AdaptiveCruise.Mode.ACCELERATE, 5.0, releasedDecision));
    }

    @Test
    void givenInvalidSpeedOrDistance_whenDistanceMethodsAreCalled_thenRejectsInput() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        assertAll(
                () -> assertThrows(IllegalArgumentException.class,
                        () -> cruise.safeDistanceFor(Double.NaN)),
                () -> assertThrows(IllegalArgumentException.class,
                        () -> cruise.update(-1.0, 1.0)),
                () -> assertThrows(IllegalArgumentException.class,
                        () -> cruise.update(1.0, -1.0)));
    }

    private void assertDecision(AdaptiveCruise.Mode expectedMode,
                                double expectedAmount,
                                AdaptiveCruise.Decision actualDecision) {
        assertAll(
                () -> assertEquals(expectedMode, actualDecision.mode()),
                () -> assertEquals(expectedAmount, actualDecision.amount(), DELTA));
    }
}
