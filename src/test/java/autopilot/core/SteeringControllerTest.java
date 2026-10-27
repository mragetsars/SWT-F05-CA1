package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.params.provider.ValueSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

@DisplayName("Steering controller - limits and self-centering")
class SteeringControllerTest {
    private static final double DELTA = 1.0e-9;
    private SteeringController controller;

    @BeforeEach
    void setUp() {
        controller = new SteeringController();
    }

    @DisplayName("Steering respects angle and per-tick limits at the given speed")
    @ParameterizedTest(name = "{displayName} [{index}]: {0}")
    @MethodSource("steeringUpdateCases")
    void givenRequestedAngleAndSpeed_whenUpdate_thenMovesWithinApplicableLimit(
            String description, double requestedAngleDegrees, double speedKmh,
            int updateCount, double expectedAngleDegrees) {
        double angle = 0.0;

        for (int update = 0; update < updateCount; update++) {
            angle = controller.update(requestedAngleDegrees, speedKmh);
        }

        assertEquals(expectedAngleDegrees, angle, DELTA, description);
    }

    @DisplayName("Reversing direction changes the angle by at most one step")
    @Tag("state")
    @ParameterizedTest(name = "{displayName} [{index}]: Reverse from {0} toward {1}")
    @CsvSource({"300.0, -300.0, 240.0, 180.0", "-300.0, 300.0, -240.0, -180.0"})
    void givenSteeredVehicle_whenOppositeDirectionRequested_thenChangesAngleByAtMostOneTick(
            double initialAngle, double requestedAngle, double firstExpectedAngle, double secondExpectedAngle) {
        moveToAngle(initialAngle, 5);

        double[] angles = {
                controller.update(requestedAngle, 0.0),
                controller.update(requestedAngle, 0.0)
        };

        assertArrayEquals(new double[]{firstExpectedAngle, secondExpectedAngle}, angles, DELTA);
    }

    @DisplayName("A nearby target is reached without overshooting")
    @Tag("state")
    @ParameterizedTest(name = "{displayName} [{index}]: Retarget from {0} to nearby {1}")
    @CsvSource({"240.0, 250.0", "-240.0, -250.0"})
    void givenCurrentAngleNearNewTarget_whenUpdate_thenReachesTargetWithoutOvershoot(
            double initialAngle, double requestedAngle) {
        moveToAngle(initialAngle, 4);

        double actualAngle = controller.update(requestedAngle, 0.0);

        assertEquals(requestedAngle, actualAngle, DELTA);
    }

    @DisplayName("One centering step approaches zero without overshooting")
    @Tag("state")
    @ParameterizedTest(name = "{displayName} [{index}]: Center one tick from {0} -> {1}")
    @CsvSource({"0.0, 0.0", "45.0, 0.0", "-45.0, 0.0", "120.0, 60.0", "-120.0, -60.0"})
    void givenSteeringOffset_whenSelfCenter_thenMovesAtMostOneTickWithoutOvershoot(
            double initialAngle, double expectedAngle) {
        moveToAngle(initialAngle, 2);

        double actualAngle = controller.selfCenter();

        assertEquals(expectedAngle, actualAngle, DELTA);
    }

    @DisplayName("Repeated centering reaches zero in bounded steps")
    @Tag("state")
    @ParameterizedTest(name = "{displayName} [{index}]: Center trajectory from {0}")
    @CsvSource({"240.0, 180.0, 120.0, 60.0", "-240.0, -180.0, -120.0, -60.0"})
    void givenSteeredVehicle_whenSelfCenterRepeatedly_thenReachesCenterInBoundedSteps(
            double initialAngle, double firstExpected, double secondExpected, double thirdExpected) {
        moveToAngle(initialAngle, 4);

        double[] angles = {
                controller.selfCenter(), controller.selfCenter(),
                controller.selfCenter(), controller.selfCenter()
        };

        assertArrayEquals(new double[]{firstExpected, secondExpected, thirdExpected, 0.0}, angles, DELTA);
    }

    @DisplayName("Increasing speed gradually reduces steering to the new limit")
    @Tag("state")
    @ParameterizedTest(name = "{displayName} [{index}]: Reduce low-speed steering {0} toward high-speed target {1}")
    @CsvSource({"300.0, 450.0, 240.0, 180.0, 120.0, 90.0",
            "-300.0, -450.0, -240.0, -180.0, -120.0, -90.0"})
    void givenLargeLowSpeedSteeringAngle_whenSpeedBecomesHigh_thenMovesTowardHighSpeedLimit(
            double initialAngle, double requestedAngle, double firstExpected,
            double secondExpected, double thirdExpected, double finalExpected) {
        moveToAngle(initialAngle, 5);

        double[] angles = new double[4];
        for (int tick = 0; tick < angles.length; tick++) {
            angles[tick] = controller.update(requestedAngle, 120.001);
        }

        assertArrayEquals(new double[]{firstExpected, secondExpected, thirdExpected, finalExpected}, angles, DELTA);
    }

    @DisplayName("Invalid angles preserve both current and target steering angles")
    @Tag("state")
    @Tag("validation")
    @ParameterizedTest(name = "{displayName} [{index}]: Reject angle {0} without changing current or pending angle")
    @ValueSource(doubles = {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void givenInvalidAngle_whenUpdate_thenRejectsInputAndPreservesSteeringState(double invalidAngle) {
        assertRejectedUpdatePreservesState(invalidAngle, 0.0);
    }

    @DisplayName("Invalid speeds preserve both current and target steering angles")
    @Tag("state")
    @Tag("validation")
    @ParameterizedTest(name = "{displayName} [{index}]: Reject speed {0} without changing current or pending angle")
    @ValueSource(doubles = {-0.1, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void givenInvalidSpeed_whenUpdate_thenRejectsInputAndPreservesSteeringState(double invalidSpeedKmh) {
        assertRejectedUpdatePreservesState(0.0, invalidSpeedKmh);
    }

    private void assertRejectedUpdatePreservesState(double requestedAngle, double speedKmh) {
        moveToAngle(240.0, 2);

        assertThrows(IllegalArgumentException.class, () -> controller.update(requestedAngle, speedKmh));

        assertAll(
                () -> assertEquals(120.0, controller.getCurrentAngleDegrees(), DELTA),
                () -> assertEquals(240.0, controller.getTargetAngleDegrees(), DELTA));
    }

    private void moveToAngle(double requestedAngleDegrees, int updates) {
        for (int update = 0; update < updates; update++) {
            controller.update(requestedAngleDegrees, 0.0);
        }
    }

    private static Stream<Arguments> steeringUpdateCases() {
        return Stream.of(
                Arguments.of("positive physical clamp at 120 km/h", 500.0, 120.0, 8, 450.0),
                Arguments.of("negative physical clamp at 120 km/h", -500.0, 120.0, 8, -450.0),
                Arguments.of("positive step just below sixty", 59.999, 0.0, 1, 59.999),
                Arguments.of("positive step at sixty", 60.0, 0.0, 1, 60.0),
                Arguments.of("positive step just above sixty", 60.001, 0.0, 1, 60.0),
                Arguments.of("negative step just below sixty", -59.999, 0.0, 1, -59.999),
                Arguments.of("negative step at sixty", -60.0, 0.0, 1, -60.0),
                Arguments.of("negative step just above sixty", -60.001, 0.0, 1, -60.0),
                Arguments.of("positive high-speed target inside limit", 89.0, 120.001, 2, 89.0),
                Arguments.of("positive high-speed target at limit", 90.0, 120.001, 2, 90.0),
                Arguments.of("positive high-speed target beyond limit", 91.0, 120.001, 2, 90.0),
                Arguments.of("negative high-speed target inside limit", -89.0, 120.001, 2, -89.0),
                Arguments.of("negative high-speed target at limit", -90.0, 120.001, 2, -90.0),
                Arguments.of("negative high-speed target beyond limit", -91.0, 120.001, 2, -90.0)
        );
    }
}
