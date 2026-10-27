package autopilot.core;

import autopilot.exception.IllegalGearChangeException;
import autopilot.exception.SensorFailureException;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;

class GearBoxTest {
    private double sensedSpeedKmh;
    private GearBox gearBox;

    @BeforeEach
    void setUp() {
        sensedSpeedKmh = 0.0;
        gearBox = new GearBox(() -> sensedSpeedKmh);
    }

    @ParameterizedTest
    @EnumSource(GearBox.Gear.class)
    void givenGearboxAlreadyInAGear_whenChangeToSameGear_thenLeavesGearUnchanged(GearBox.Gear gear) {
        moveTo(gear);

        gearBox.changeTo(gear);

        assertEquals(gear, gearBox.getCurrentGear());
    }

    @Test
    void givenStoppedVehicle_whenChangingThroughLegalSequence_thenReachesDrive() {
        gearBox.changeTo(GearBox.Gear.R);
        gearBox.changeTo(GearBox.Gear.N);
        gearBox.changeTo(GearBox.Gear.D);

        assertEquals(GearBox.Gear.D, gearBox.getCurrentGear());
    }

    @Test
    void givenVehicleBelowReverseEntryLimit_whenChangeToReverse_thenAllowsTransition() {
        sensedSpeedKmh = 4.999;

        gearBox.changeTo(GearBox.Gear.R);

        assertEquals(GearBox.Gear.R, gearBox.getCurrentGear());
    }

    @Test
    void givenVehicleAtReverseEntryLimit_whenChangeToReverse_thenRejectsTransition() {
        sensedSpeedKmh = 5.0;

        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(GearBox.Gear.R));

        assertEquals(GearBox.Gear.P, gearBox.getCurrentGear());
    }

    @Test
    void givenVehicleAboveReverseEntryLimit_whenChangeToReverse_thenRejectsTransition() {
        sensedSpeedKmh = 5.001;

        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(GearBox.Gear.R));

        assertEquals(GearBox.Gear.P, gearBox.getCurrentGear());
    }

    @Test
    void givenStoppedVehicleInReverse_whenChangeToDrive_thenAllowsTransition() {
        gearBox.changeTo(GearBox.Gear.R);

        gearBox.changeTo(GearBox.Gear.D);

        assertEquals(GearBox.Gear.D, gearBox.getCurrentGear());
    }

    @Test
    void givenMovingVehicleInReverse_whenChangeToDrive_thenRejectsTransitionAndKeepsReverse() {
        gearBox.changeTo(GearBox.Gear.R);
        sensedSpeedKmh = 0.001;

        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(GearBox.Gear.D));

        assertEquals(GearBox.Gear.R, gearBox.getCurrentGear());
    }

    @Test
    void givenStoppedVehicleInDrive_whenChangeToReverse_thenAllowsTransition() {
        moveTo(GearBox.Gear.D);

        gearBox.changeTo(GearBox.Gear.R);

        assertEquals(GearBox.Gear.R, gearBox.getCurrentGear());
    }

    @Test
    void givenMovingVehicleInDrive_whenChangeToReverse_thenRejectsTransitionAndKeepsDrive() {
        moveTo(GearBox.Gear.D);
        sensedSpeedKmh = 0.001;

        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(GearBox.Gear.R));

        assertEquals(GearBox.Gear.D, gearBox.getCurrentGear());
    }

    @Test
    void givenParkGear_whenChangeToNeutral_thenRejectsSkippedTransition() {
        assertRejectedTransition(GearBox.Gear.N, GearBox.Gear.P);
    }

    @Test
    void givenParkGear_whenChangeToDrive_thenRejectsSkippedTransition() {
        assertRejectedTransition(GearBox.Gear.D, GearBox.Gear.P);
    }

    @Test
    void givenReverseGear_whenChangeToPark_thenRejectsSkippedTransition() {
        moveTo(GearBox.Gear.R);

        assertRejectedTransition(GearBox.Gear.P, GearBox.Gear.R);
    }

    @Test
    void givenNeutralGear_whenChangeToPark_thenRejectsSkippedTransition() {
        moveTo(GearBox.Gear.N);

        assertRejectedTransition(GearBox.Gear.P, GearBox.Gear.N);
    }

    @Test
    void givenNeutralGear_whenChangeToReverse_thenRejectsSkippedTransition() {
        moveTo(GearBox.Gear.N);

        assertRejectedTransition(GearBox.Gear.R, GearBox.Gear.N);
    }

    @Test
    void givenDriveGear_whenChangeToNeutral_thenRejectsSkippedTransition() {
        moveTo(GearBox.Gear.D);

        assertRejectedTransition(GearBox.Gear.N, GearBox.Gear.D);
    }

    @Test
    void givenDriveGear_whenChangeToPark_thenRejectsSkippedTransition() {
        moveTo(GearBox.Gear.D);

        assertRejectedTransition(GearBox.Gear.P, GearBox.Gear.D);
    }

    @Test
    void givenInvalidSensorReading_whenChangeToReverse_thenRejectsTransitionAndKeepsPark() {
        sensedSpeedKmh = Double.NaN;

        assertThrows(IllegalArgumentException.class, () -> gearBox.changeTo(GearBox.Gear.R));

        assertEquals(GearBox.Gear.P, gearBox.getCurrentGear());
    }

    @Test
    void givenNegativeOrInfiniteSensorReading_whenChangeToReverse_thenRejectsTransitionAndKeepsPark() {
        assertAll(
                () -> assertThrows(IllegalArgumentException.class, () -> changeToReverseAt(-0.1)),
                () -> assertThrows(IllegalArgumentException.class,
                        () -> changeToReverseAt(Double.POSITIVE_INFINITY)),
                () -> assertThrows(IllegalArgumentException.class,
                        () -> changeToReverseAt(Double.NEGATIVE_INFINITY)));

        assertEquals(GearBox.Gear.P, gearBox.getCurrentGear());
    }

    @Test
    void givenReverseGear_whenSensorReadingBecomesInvalid_thenRejectsTransitionAndKeepsReverse() {
        moveTo(GearBox.Gear.R);
        sensedSpeedKmh = Double.POSITIVE_INFINITY;

        assertThrows(IllegalArgumentException.class, () -> gearBox.changeTo(GearBox.Gear.D));

        assertEquals(GearBox.Gear.R, gearBox.getCurrentGear());
    }

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

    @Test
    void givenFailedSpeedSensor_whenChangeToCurrentGear_thenDoesNotReadSensor() {
        gearBox.setSpeedSensor(() -> {
            throw new SensorFailureException("Speed sensor failed");
        });

        gearBox.changeTo(GearBox.Gear.P);

        assertEquals(GearBox.Gear.P, gearBox.getCurrentGear());
    }

    @Test
    void givenNullTargetGear_whenChangeTo_thenRejectsInput() {
        assertThrows(NullPointerException.class, () -> gearBox.changeTo(null));

        assertEquals(GearBox.Gear.P, gearBox.getCurrentGear());
    }

    private void assertRejectedTransition(GearBox.Gear targetGear, GearBox.Gear expectedGear) {
        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(targetGear));

        assertEquals(expectedGear, gearBox.getCurrentGear());
    }

    private void changeToReverseAt(double speedKmh) {
        sensedSpeedKmh = speedKmh;
        gearBox.changeTo(GearBox.Gear.R);
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
