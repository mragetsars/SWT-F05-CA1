package autopilot.core;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvFileSource;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.EnumSource;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class BrakeControllerTest {
    private static final double ABS_TRIGGER_PRESSURE = 81.0;
    private static final double ABS_TRIGGER_SPEED_KMH = 31.0;
    private static final double ABS_HIGH_PULSE = 90.0;
    private static final double ABS_LOW_PULSE = 60.0;

    @ParameterizedTest
    @CsvFileSource(resources = "/brake-application-cases.csv", numLinesToSkip = 1)
    void givenOrdinaryBrakingConditions_whenApply_thenReturnsBoundedRequestedPressure(
            double requestedPressure,
            double speedKmh,
            double expectedPressure) {
        BrakeController controller = new BrakeController();

        double pressure = controller.apply(requestedPressure, speedKmh, GearBox.Gear.D);

        assertAll(
                () -> assertEquals(expectedPressure, pressure),
                () -> assertEquals(expectedPressure, controller.getPressure()),
                () -> assertFalse(controller.isWarningActive()));
    }

    @ParameterizedTest
    @CsvSource({
            "80.0, 31.0, false, 80.0",
            "80.0001, 30.0, false, 80.0001",
            "80.0001, 30.0001, true, 0.0",
            "100.0, 31.0, true, 0.0",
            "150.0, 31.0, true, 0.0"
    })
    void givenInputsAroundAbsThresholds_whenApply_thenActivatesAbsOnlyAboveBothThresholds(
            double requestedPressure,
            double speedKmh,
            boolean absExpected,
            double expectedOrdinaryPressure) {
        BrakeController controller = new BrakeController();

        double pressure = controller.apply(requestedPressure, speedKmh, GearBox.Gear.D);

        if (absExpected) {
            assertTrue(isAbsPulse(pressure));
        } else {
            assertEquals(expectedOrdinaryPressure, pressure);
        }
    }

    @Test
    void givenHardBrakingAboveAbsThreshold_whenApplyRepeatedly_thenAlternatesAbsPulsesAndResetsAfterNormalBrake() {
        BrakeController controller = new BrakeController();

        double firstPulse = controller.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);
        double secondPulse = controller.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);
        controller.apply(70.0, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);
        double pulseAfterReset = controller.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);

        assertAll(
                () -> assertTrue(isAbsPulse(firstPulse)),
                () -> assertTrue(isAbsPulse(secondPulse)),
                () -> assertNotEquals(firstPulse, secondPulse),
                () -> assertEquals(firstPulse, pulseAfterReset));
    }

    @ParameterizedTest
    @CsvSource({"R", "N", "D"})
    void givenBrakingAllowedGear_whenEmergencyBrake_thenAppliesMaximumPressureAndActivatesWarning(
            GearBox.Gear gear) {
        BrakeController controller = new BrakeController();

        controller.emergencyBrake(gear);

        assertAll(
                () -> assertEquals(100.0, controller.getPressure()),
                () -> assertTrue(controller.isWarningActive()));
    }

    @Test
    void givenParkGear_whenApply_thenRejectsBrakingWithoutChangingState() {
        BrakeController controller = new BrakeController();

        assertThrows(IllegalStateException.class,
                () -> controller.apply(10.0, 0.0, GearBox.Gear.P));

        assertAll(
                () -> assertEquals(0.0, controller.getPressure()),
                () -> assertFalse(controller.isWarningActive()));
    }

    @Test
    void givenParkGear_whenEmergencyBrake_thenRejectsBrakingWithoutActivatingWarning() {
        BrakeController controller = new BrakeController();

        assertThrows(IllegalStateException.class, () -> controller.emergencyBrake(GearBox.Gear.P));

        assertAll(
                () -> assertEquals(0.0, controller.getPressure()),
                () -> assertFalse(controller.isWarningActive()));
    }

    @ParameterizedTest
    @EnumSource(value = GearBox.Gear.class, names = "P", mode = EnumSource.Mode.EXCLUDE)
    void givenNonParkGear_whenApply_thenAllowsOrdinaryBraking(GearBox.Gear gear) {
        BrakeController controller = new BrakeController();

        double pressure = controller.apply(40.0, 0.0, gear);

        assertEquals(40.0, pressure);
    }

    @ParameterizedTest
    @CsvSource({"-0.1", "NaN", "Infinity", "-Infinity"})
    void givenInvalidRequestedPressure_whenApply_thenRejectsInput(double requestedPressure) {
        BrakeController controller = new BrakeController();

        assertThrows(IllegalArgumentException.class,
                () -> controller.apply(requestedPressure, 0.0, GearBox.Gear.D));
    }

    @ParameterizedTest
    @CsvSource({"-0.1", "NaN", "Infinity", "-Infinity"})
    void givenInvalidSpeed_whenApply_thenRejectsInput(double speedKmh) {
        BrakeController controller = new BrakeController();

        assertThrows(IllegalArgumentException.class,
                () -> controller.apply(0.0, speedKmh, GearBox.Gear.D));
    }

    @Test
    void givenNullGear_whenBrakingMethodsAreCalled_thenRejectsInput() {
        BrakeController controller = new BrakeController();

        assertAll(
                () -> assertThrows(NullPointerException.class,
                        () -> controller.apply(10.0, 0.0, null)),
                () -> assertThrows(NullPointerException.class,
                        () -> controller.emergencyBrake(null)));
    }

    @Test
    void givenEmergencyBrakeAfterAbsCycle_whenHardBrakingResumes_thenRestartsWithInitialAbsPulse() {
        BrakeController controller = new BrakeController();
        double firstPulse = controller.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);
        controller.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);
        controller.emergencyBrake(GearBox.Gear.D);

        double pressure = controller.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);

        assertEquals(firstPulse, pressure);
    }

    @Test
    void givenActiveAbsCycle_whenApplyRejectsInvalidInput_thenPreservesPressureAndAbsPhase() {
        BrakeController controller = new BrakeController();
        double firstPulse = controller.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);

        assertThrows(IllegalArgumentException.class,
                () -> controller.apply(Double.NaN, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D));
        assertEquals(firstPulse, controller.getPressure());
        double nextPulse = controller.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);

        assertAll(
                () -> assertTrue(isAbsPulse(nextPulse)),
                () -> assertNotEquals(firstPulse, nextPulse));
    }

    @Test
    void givenEmergencyBrakeState_whenParkBrakeIsRejected_thenPreservesEmergencyState() {
        BrakeController controller = new BrakeController();
        controller.emergencyBrake(GearBox.Gear.D);

        assertThrows(IllegalStateException.class,
                () -> controller.apply(10.0, 0.0, GearBox.Gear.P));

        assertAll(
                () -> assertEquals(100.0, controller.getPressure()),
                () -> assertTrue(controller.isWarningActive()));
    }

    @Test
    void givenActiveAbsCycle_whenParkBrakeIsRejected_thenPreservesAbsPhase() {
        BrakeController controller = new BrakeController();
        double firstPulse = controller.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);

        assertThrows(IllegalStateException.class,
                () -> controller.apply(10.0, 0.0, GearBox.Gear.P));
        double nextPulse = controller.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);

        assertAll(
                () -> assertTrue(isAbsPulse(nextPulse)),
                () -> assertNotEquals(firstPulse, nextPulse));
    }

    private boolean isAbsPulse(double pressure) {
        return pressure == ABS_HIGH_PULSE || pressure == ABS_LOW_PULSE;
    }
}
