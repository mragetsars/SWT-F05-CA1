package autopilot.core;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Tag;
import autopilot.exception.IllegalGearChangeException;
import autopilot.exception.SensorFailureException;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;

@DisplayName("Gearbox - transitions and state preservation")
@Tag("state")
class GearBoxTest {
    private double sensedSpeedKmh;
    private GearBox gearBox;

    @BeforeEach
    void setUp() {
        sensedSpeedKmh = 0.0;
        gearBox = new GearBox(() -> sensedSpeedKmh);
    }

    @DisplayName("Selecting the current gear preserves it without reading the sensor")
    @ParameterizedTest(name = "{displayName} [{index}]: Already in {0}; sensor must not be read")
    @EnumSource(GearBox.Gear.class)
    void givenGearboxAlreadyInAGear_whenChangeToSameGear_thenLeavesGearUnchangedWithoutReadingSensor(
            GearBox.Gear gear) {
        moveTo(gear);
        gearBox.setSpeedSensor(() -> {
            throw new AssertionError("A no-op shift must not read the speed sensor");
        });

        gearBox.changeTo(gear);

        assertEquals(gear, gearBox.getCurrentGear());
    }

    @DisplayName("Allowed shifts select the requested gear")
    @ParameterizedTest(name = "{displayName} [{index}]: Allow {0} -> {1} at {2} km/h")
    @CsvSource({
            "P, R, 4.999",
            "R, D, 0.0",
            "D, R, 0.0",
            "R, N, 30.0",
            "N, D, 30.0"
    })
    void givenAllowedGearAndSpeed_whenChangeTo_thenSelectsRequestedGear(
            GearBox.Gear initialGear, GearBox.Gear targetGear, double speedKmh) {
        moveTo(initialGear);
        sensedSpeedKmh = speedKmh;

        gearBox.changeTo(targetGear);

        assertEquals(targetGear, gearBox.getCurrentGear());
    }

    @DisplayName("Forbidden shifts preserve the current gear")
    @Tag("validation")
    @ParameterizedTest(name = "{displayName} [{index}]: Reject {0} -> {1} at {2} km/h")
    @CsvSource({
            "P, R, 5.0",
            "P, R, 5.001",
            "R, D, 0.001",
            "D, R, 0.001",
            "P, N, 0.0",
            "P, D, 0.0",
            "R, P, 0.0",
            "N, P, 0.0",
            "N, R, 0.0",
            "D, N, 0.0",
            "D, P, 0.0"
    })
    void givenForbiddenGearOrSpeed_whenChangeTo_thenRejectsTransitionAndKeepsGear(
            GearBox.Gear initialGear, GearBox.Gear targetGear, double speedKmh) {
        moveTo(initialGear);
        sensedSpeedKmh = speedKmh;

        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(targetGear));

        assertEquals(initialGear, gearBox.getCurrentGear());
    }

    @DisplayName("Invalid speed readings preserve reverse gear")
    @Tag("validation")
    @ParameterizedTest(name = "{displayName} [{index}]: Reject invalid speed {0} without changing reverse gear")
    @ValueSource(doubles = {-0.1, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void givenInvalidSensorReading_whenChangeToDrive_thenRejectsTransitionAndKeepsReverse(double speedKmh) {
        moveTo(GearBox.Gear.R);
        sensedSpeedKmh = speedKmh;

        assertThrows(IllegalArgumentException.class, () -> gearBox.changeTo(GearBox.Gear.D));

        assertEquals(GearBox.Gear.R, gearBox.getCurrentGear());
    }

    @DisplayName("Speed-sensor failures propagate without changing gear")
    @Test
    void givenFailedSpeedSensor_whenChangeRequiresSpeed_thenPropagatesFailureAndKeepsGear() {
        moveTo(GearBox.Gear.R);
        SensorFailureException expectedFailure = new SensorFailureException("Speed sensor failed");
        gearBox.setSpeedSensor(() -> {
            throw expectedFailure;
        });

        SensorFailureException actualFailure = assertThrows(
                SensorFailureException.class, () -> gearBox.changeTo(GearBox.Gear.D));

        assertAll(
                () -> assertSame(expectedFailure, actualFailure),
                () -> assertEquals(GearBox.Gear.R, gearBox.getCurrentGear()));
    }

    @DisplayName("A null target gear is rejected without resetting the current gear")
    @Tag("validation")
    @Test
    void givenNullTargetGear_whenChangeTo_thenRejectsInputWithoutResettingExistingGear() {
        moveTo(GearBox.Gear.D);

        assertThrows(NullPointerException.class, () -> gearBox.changeTo(null));

        assertEquals(GearBox.Gear.D, gearBox.getCurrentGear());
    }

    private void moveTo(GearBox.Gear gear) {
        switch (gear) {
            case P -> {
                return;
            }
            case R -> gearBox.changeTo(GearBox.Gear.R);
            case N -> {
                gearBox.changeTo(GearBox.Gear.R);
                gearBox.changeTo(GearBox.Gear.N);
            }
            case D -> {
                gearBox.changeTo(GearBox.Gear.R);
                gearBox.changeTo(GearBox.Gear.N);
                gearBox.changeTo(GearBox.Gear.D);
            }
        }
    }
}
