package autopilot.core;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvFileSource;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
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
                () -> assertEquals(expectedPressure, controller.getPressure()));
    }

    @Test
    void givenHardBrakingAboveAbsThreshold_whenApplyRepeatedly_thenAlternatesAbsPulses() {
        BrakeController controller = new BrakeController();

        double firstPulse = controller.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);
        double secondPulse = controller.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);
        controller.apply(70.0, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);
        double pulseAfterReset = controller.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);

        assertAll(
                () -> assertEquals(ABS_HIGH_PULSE, firstPulse),
                () -> assertEquals(ABS_LOW_PULSE, secondPulse),
                () -> assertEquals(ABS_HIGH_PULSE, pulseAfterReset));
    }

    @Test
    void givenDriveGear_whenEmergencyBrake_thenAppliesMaximumPressureAndActivatesWarning() {
        BrakeController controller = new BrakeController();

        controller.emergencyBrake(GearBox.Gear.D);

        assertAll(
                () -> assertEquals(100.0, controller.getPressure()),
                () -> assertTrue(controller.isWarningActive()));
    }

    @Test
    void givenParkGear_whenApply_thenRejectsBraking() {
        BrakeController controller = new BrakeController();

        assertThrows(IllegalStateException.class,
                () -> controller.apply(10.0, 0.0, GearBox.Gear.P));
    }

    @Test
    void givenInvalidPressureOrSpeed_whenApply_thenRejectsInput() {
        BrakeController controller = new BrakeController();

        assertAll(
                () -> assertThrows(IllegalArgumentException.class,
                        () -> controller.apply(-1.0, 0.0, GearBox.Gear.D)),
                () -> assertThrows(IllegalArgumentException.class,
                        () -> controller.apply(0.0, Double.POSITIVE_INFINITY, GearBox.Gear.D)));
    }
}
