package autopilot.core;

import autopilot.exception.SensorFailureException;
import autopilot.sensor.LaneSensor;
import autopilot.sensor.ObstacleSensor;
import autopilot.sensor.SpeedSensor;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class AutoDriverTest {
    private static final double CRUISING_SPEED_KMH = 80.0;
    private static final double CLEAR_ROAD_DISTANCE_METERS = 100.0;
    private static final double TARGET_SPEED_KMH = 80.0;
    private static final double LANE_CORRECTION_THRESHOLD_METERS = 0.5;
    private static final double LANE_CORRECTION_DEGREES = 45.0;
    private static final double SAFE_MODE_BRAKE_PRESSURE = 30.0;

    @Test
    void givenClearRoadAndCenteredLane_whenTickInDrive_thenReturnsActiveHoldResult() {
        AutoDriver autoDriver = createAutoDriver(
                () -> CRUISING_SPEED_KMH,
                () -> CLEAR_ROAD_DISTANCE_METERS,
                () -> 0.0);

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertAll(
                () -> assertFalse(result.safeMode()),
                () -> assertEquals(AdaptiveCruise.Mode.HOLD, result.cruiseMode()),
                () -> assertEquals(0.0, result.steeringAngleDegrees()),
                () -> assertEquals(0.0, result.brakePressure()),
                () -> assertFalse(result.warningActive()));
    }

    @ParameterizedTest
    @ValueSource(doubles = {
            LANE_CORRECTION_THRESHOLD_METERS + 0.5,
            -(LANE_CORRECTION_THRESHOLD_METERS + 0.5)
    })
    void givenLaneDeviationBeyondThreshold_whenTickInDrive_thenSteersTowardLaneCenter(double laneOffsetMeters) {
        AutoDriver autoDriver = createAutoDriver(
                () -> CRUISING_SPEED_KMH,
                () -> CLEAR_ROAD_DISTANCE_METERS,
                () -> laneOffsetMeters);

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertEquals(-Math.copySign(LANE_CORRECTION_DEGREES, laneOffsetMeters),
                result.steeringAngleDegrees());
    }

    @ParameterizedTest
    @ValueSource(doubles = {
            LANE_CORRECTION_THRESHOLD_METERS,
            -LANE_CORRECTION_THRESHOLD_METERS
    })
    void givenLaneDeviationAtThreshold_whenTickInDrive_thenDoesNotCorrectSteering(double laneOffsetMeters) {
        AutoDriver autoDriver = createAutoDriver(
                () -> CRUISING_SPEED_KMH,
                () -> CLEAR_ROAD_DISTANCE_METERS,
                () -> laneOffsetMeters);

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertEquals(0.0, result.steeringAngleDegrees());
    }

    @Test
    void givenObstacleInsideSafeDistance_whenTickInDrive_thenAppliesProportionalBraking() {
        AutoDriver autoDriver = createAutoDriver(
                () -> CRUISING_SPEED_KMH,
                () -> 30.0,
                () -> 0.0);

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertAll(
                () -> assertFalse(result.safeMode()),
                () -> assertEquals(AdaptiveCruise.Mode.BRAKE, result.cruiseMode()),
                () -> assertEquals(25.0, result.brakePressure()),
                () -> assertFalse(result.warningActive()));
    }

    @Test
    void givenObstacleAtEmergencyBoundary_whenTickInDrive_thenActivatesEmergencyBrakingAndWarning() {
        AutoDriver autoDriver = createAutoDriver(
                () -> CRUISING_SPEED_KMH,
                () -> 20.0,
                () -> 0.0);

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertAll(
                () -> assertFalse(result.safeMode()),
                () -> assertEquals(AdaptiveCruise.Mode.EMERGENCY_BRAKE, result.cruiseMode()),
                () -> assertEquals(100.0, result.brakePressure()),
                () -> assertTrue(result.warningActive()));
    }

    @Test
    void givenCorrectedLanePosition_whenNextTickIsCentered_thenReturnsSteeringToCenter() {
        double[] laneOffsetMeters = {1.0};
        AutoDriver autoDriver = createAutoDriver(
                () -> CRUISING_SPEED_KMH,
                () -> CLEAR_ROAD_DISTANCE_METERS,
                () -> laneOffsetMeters[0]);

        AutoDriver.DriveResult correctedResult = autoDriver.tick(GearBox.Gear.D);
        laneOffsetMeters[0] = 0.0;
        AutoDriver.DriveResult centeredResult = autoDriver.tick(GearBox.Gear.D);

        assertAll(
                () -> assertEquals(-LANE_CORRECTION_DEGREES, correctedResult.steeringAngleDegrees()),
                () -> assertEquals(0.0, centeredResult.steeringAngleDegrees()));
    }

    @ParameterizedTest(name = "Safe mode after {0} sensor failure")
    @EnumSource(FailedSensor.class)
    void givenSensorFailure_whenTickInDrive_thenEntersSafeMode(FailedSensor failedSensor) {
        SpeedSensor speedSensor = failedSensor == FailedSensor.SPEED
                ? AutoDriverTest::failedReading : () -> CRUISING_SPEED_KMH;
        ObstacleSensor obstacleSensor = failedSensor == FailedSensor.OBSTACLE
                ? AutoDriverTest::failedReading : () -> CLEAR_ROAD_DISTANCE_METERS;
        LaneSensor laneSensor = failedSensor == FailedSensor.LANE
                ? AutoDriverTest::failedReading : () -> 0.0;

        assertSafeModeResult(speedSensor, obstacleSensor, laneSensor);
    }

    private AutoDriver createAutoDriver(SpeedSensor speedSensor,
                                        ObstacleSensor obstacleSensor,
                                        LaneSensor laneSensor) {
        return new AutoDriver(speedSensor, obstacleSensor, laneSensor, TARGET_SPEED_KMH);
    }

    private void assertSafeModeResult(SpeedSensor speedSensor,
                                      ObstacleSensor obstacleSensor,
                                      LaneSensor laneSensor) {
        AutoDriver autoDriver = createAutoDriver(speedSensor, obstacleSensor, laneSensor);

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertAll(
                () -> assertEquals(0.0, result.steeringAngleDegrees()),
                () -> assertEquals(SAFE_MODE_BRAKE_PRESSURE, result.brakePressure()),
                () -> assertEquals(AdaptiveCruise.Mode.BRAKE, result.cruiseMode()),
                () -> assertFalse(result.warningActive()),
                () -> assertTrue(result.safeMode()));
    }

    private static double failedReading() {
        throw new SensorFailureException("Sensor unavailable");
    }

    private enum FailedSensor {
        SPEED, OBSTACLE, LANE
    }
}
