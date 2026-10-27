package autopilot.core;

import autopilot.exception.SensorFailureException;
import autopilot.sensor.LaneSensor;
import autopilot.sensor.ObstacleSensor;
import autopilot.sensor.SensorSnapshot;
import autopilot.sensor.SpeedSensor;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
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

    @Test
    void givenSpeedSensorFailure_whenTickInDrive_thenEntersSafeMode() {
        assertSafeModeResult(
                () -> {
                    throw new SensorFailureException("Speed sensor failed");
                },
                () -> CLEAR_ROAD_DISTANCE_METERS,
                () -> 0.0);
    }

    @Test
    void givenObstacleSensorFailure_whenTickInDrive_thenEntersSafeMode() {
        assertSafeModeResult(
                () -> CRUISING_SPEED_KMH,
                () -> {
                    throw new SensorFailureException("Obstacle sensor failed");
                },
                () -> 0.0);
    }

    @Test
    void givenLaneSensorFailure_whenTickInDrive_thenEntersSafeMode() {
        assertSafeModeResult(
                () -> CRUISING_SPEED_KMH,
                () -> CLEAR_ROAD_DISTANCE_METERS,
                () -> {
                    throw new SensorFailureException("Lane sensor failed");
                });
    }

    @Test
    void givenFailedSensorSnapshot_whenTickInDrive_thenEntersSafeMode() {
        SensorSnapshot failedSnapshot = SensorSnapshot.failed("Sensor snapshot failed");

        assertSafeModeResult(failedSnapshot, failedSnapshot, failedSnapshot);
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
}
