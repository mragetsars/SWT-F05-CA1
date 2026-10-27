package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.CsvFileSource;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.params.provider.ValueSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

@DisplayName("Brake controller - pressure, ABS, and emergency braking")
class BrakeControllerTest {
    private static final double ABS_TRIGGER_PRESSURE = 81.0;
    private static final double ABS_TRIGGER_SPEED_KMH = 31.0;
    private static final double ABS_HIGH_PULSE = 90.0;
    private static final double ABS_LOW_PULSE = 60.0;
    private static final double DELTA = 1.0e-9;

    private BrakeController controller;

    @BeforeEach
    void setUp() {
        controller = new BrakeController();
    }

    @DisplayName("Ordinary braking returns and stores bounded pressure")
    @ParameterizedTest(name = "{displayName} [{index}]: pressure={0}, speed={1} -> pressure={2}")
    @CsvFileSource(resources = "/brake-application-cases.csv", numLinesToSkip = 1)
    void givenOrdinaryBrakingConditions_whenApply_thenReturnsAndStoresBoundedPressure(
            double requestedPressure, double speedKmh, double expectedPressure) {
        double pressure = controller.apply(requestedPressure, speedKmh, GearBox.Gear.D);

        assertAll(
                () -> assertEquals(expectedPressure, pressure, DELTA),
                () -> assertEquals(expectedPressure, controller.getPressure(), DELTA),
                () -> assertFalse(controller.isWarningActive()));
    }

    @DisplayName("ABS starts with a high pulse only above both thresholds")
    @ParameterizedTest(name = "{displayName} [{index}]: ABS boundary: pressure={0}, speed={1} -> {2}")
    @CsvSource({
            "80.0,    31.0,    80.0",
            "80.0001, 30.0,    80.0001",
            "80.0001, 30.0001, 90.0",
            "150.0,   31.0,    90.0"
    })
    void givenInputsAroundAbsThresholds_whenApply_thenStartsHighPulseOnlyAboveBothThresholds(
            double requestedPressure, double speedKmh, double expectedPressure) {
        double pressure = controller.apply(requestedPressure, speedKmh, GearBox.Gear.D);

        assertEquals(expectedPressure, pressure, DELTA);
    }

    @DisplayName("Continuous ABS pulses alternate 90, 60, 90, 60")
    @Tag("state")
    @Test
    void givenContinuousHardBraking_whenApplyRepeatedly_thenAlternatesStartingWithHighPulse() {
        double[] pulses = new double[4];

        for (int tick = 0; tick < pulses.length; tick++) {
            pulses[tick] = controller.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);
        }

        assertArrayEquals(new double[]{90.0, 60.0, 90.0, 60.0}, pulses, DELTA);
    }

    @DisplayName("Leaving ABS resets the next cycle to the high pulse")
    @Tag("state")
    @ParameterizedTest(name = "{displayName} [{index}]: Reset ABS through pressure={0}, speed={1}")
    @CsvSource({"80.0, 31.0", "81.0, 30.0"})
    void givenActiveAbsCycle_whenOrdinaryBrakingOccurs_thenRestartsWithHighPulse(
            double ordinaryPressure, double ordinarySpeedKmh) {
        controller.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);
        controller.apply(ordinaryPressure, ordinarySpeedKmh, GearBox.Gear.D);

        double pulseAfterReset = controller.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);

        assertEquals(ABS_HIGH_PULSE, pulseAfterReset, DELTA);
    }

    @DisplayName("Emergency braking applies full pressure and activates the warning")
    @ParameterizedTest(name = "{displayName} [{index}]: Emergency brake in {0}")
    @EnumSource(value = GearBox.Gear.class, names = "P", mode = EnumSource.Mode.EXCLUDE)
    void givenBrakingAllowedGear_whenEmergencyBrake_thenAppliesMaximumPressureAndActivatesWarning(
            GearBox.Gear gear) {
        controller.emergencyBrake(gear);

        assertAll(
                () -> assertEquals(100.0, controller.getPressure(), DELTA),
                () -> assertTrue(controller.isWarningActive()));
    }

    @DisplayName("Rejected emergency braking in park preserves the brake state")
    @Tag("state")
    @Tag("validation")
    @Test
    void givenActiveAbsCycle_whenEmergencyBrakeInParkIsRejected_thenPreservesBrakeState() {
        controller.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);

        assertThrows(IllegalStateException.class, () -> controller.emergencyBrake(GearBox.Gear.P));

        assertAll(
                () -> assertEquals(ABS_HIGH_PULSE, controller.getPressure(), DELTA),
                () -> assertFalse(controller.isWarningActive()));
        assertEquals(ABS_LOW_PULSE,
                controller.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D), DELTA);
    }

    @DisplayName("Ordinary braking is allowed in every non-park gear")
    @ParameterizedTest(name = "{displayName} [{index}]: Ordinary brake in {0}")
    @EnumSource(value = GearBox.Gear.class, names = "P", mode = EnumSource.Mode.EXCLUDE)
    void givenNonParkGear_whenApply_thenAllowsOrdinaryBraking(GearBox.Gear gear) {
        double pressure = controller.apply(40.0, 0.0, gear);

        assertEquals(40.0, pressure, DELTA);
    }

    @DisplayName("Invalid requested pressure is rejected")
    @Tag("validation")
    @ParameterizedTest(name = "{displayName} [{index}]: Reject requested pressure {0}")
    @ValueSource(doubles = {-0.1, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void givenInvalidRequestedPressure_whenApply_thenRejectsInput(double requestedPressure) {
        assertThrows(IllegalArgumentException.class,
                () -> controller.apply(requestedPressure, 0.0, GearBox.Gear.D));
    }

    @DisplayName("Invalid speed is rejected")
    @Tag("validation")
    @ParameterizedTest(name = "{displayName} [{index}]: Reject speed {0}")
    @ValueSource(doubles = {-0.1, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void givenInvalidSpeed_whenApply_thenRejectsInput(double speedKmh) {
        assertThrows(IllegalArgumentException.class,
                () -> controller.apply(0.0, speedKmh, GearBox.Gear.D));
    }

    @DisplayName("Emergency braking rejects a null gear")
    @Tag("validation")
    @Test
    void givenNullGear_whenEmergencyBrake_thenRejectsInput() {
        assertThrows(NullPointerException.class, () -> controller.emergencyBrake(null));
    }

    @DisplayName("Emergency braking resets the next ABS pulse to high")
    @Tag("state")
    @Test
    void givenActiveAbsCycle_whenEmergencyBrakeOccurs_thenRestartsWithHighPulse() {
        controller.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);

        controller.emergencyBrake(GearBox.Gear.D);
        double pressure = controller.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);

        assertEquals(ABS_HIGH_PULSE, pressure, DELTA);
    }

    @DisplayName("Rejected braking preserves pressure, warning, and ABS phase")
    @Tag("state")
    @Tag("validation")
    @ParameterizedTest(name = "{displayName} [{index}]: Rejected application: {0}")
    @MethodSource("rejectedApplications")
    void givenActiveAbsCycle_whenApplyIsRejected_thenPreservesPressureWarningAndPulsePhase(
            String description, double requestedPressure, double speedKmh, GearBox.Gear gear,
            Class<? extends RuntimeException> expectedException) {
        controller.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);

        assertThrows(expectedException,
                () -> controller.apply(requestedPressure, speedKmh, gear), description);
        assertAll(
                () -> assertEquals(ABS_HIGH_PULSE, controller.getPressure(), DELTA),
                () -> assertFalse(controller.isWarningActive()));
        assertEquals(ABS_LOW_PULSE,
                controller.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D), DELTA);
    }

    @DisplayName("Rejected braking in park preserves the previous emergency state")
    @Tag("state")
    @Tag("validation")
    @Test
    void givenEmergencyBrakeState_whenParkBrakeIsRejected_thenPreservesEmergencyState() {
        controller.emergencyBrake(GearBox.Gear.D);

        assertThrows(IllegalStateException.class,
                () -> controller.apply(10.0, 0.0, GearBox.Gear.P));

        assertAll(
                () -> assertEquals(100.0, controller.getPressure(), DELTA),
                () -> assertTrue(controller.isWarningActive()));
    }

    private static Stream<Arguments> rejectedApplications() {
        return Stream.of(
                Arguments.of("invalid pressure", Double.NaN, 31.0, GearBox.Gear.D, IllegalArgumentException.class),
                Arguments.of("invalid speed", 81.0, Double.NaN, GearBox.Gear.D, IllegalArgumentException.class),
                Arguments.of("park gear", 10.0, 0.0, GearBox.Gear.P, IllegalStateException.class),
                Arguments.of("null gear", 10.0, 0.0, null, NullPointerException.class)
        );
    }
}
