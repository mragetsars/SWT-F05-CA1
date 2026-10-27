package autopilot.core;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class SteeringControllerTest {
    @ParameterizedTest
    @MethodSource("steeringUpdateCases")
    void givenRequestedAngleAndSpeed_whenUpdate_thenMovesWithinApplicableLimit(double requestedAngleDegrees,
                                                                                 double speedKmh,
                                                                                 int updateCount,
                                                                                 double expectedAngleDegrees) {
        SteeringController controller = new SteeringController();

        double angle = 0.0;
        for (int update = 0; update < updateCount; update++) {
            angle = controller.update(requestedAngleDegrees, speedKmh);
        }

        assertEquals(expectedAngleDegrees, angle);
    }

    @Test
    void givenSteeredVehicle_whenOppositeDirectionRequested_thenChangesAngleByAtMostOneTick() {
        SteeringController controller = new SteeringController();
        moveToAngle(controller, 300.0, 5);

        double firstOppositeStep = controller.update(-300.0, 0.0);
        double secondOppositeStep = controller.update(-300.0, 0.0);

        assertAll(
                () -> assertEquals(240.0, firstOppositeStep),
                () -> assertEquals(180.0, secondOppositeStep));
    }

    @Test
    void givenSteeredVehicle_whenSelfCenter_thenReturnsInBoundedStepsWithoutOvershoot() {
        SteeringController controller = new SteeringController();
        moveToAngle(controller, 240.0, 4);

        double firstCenterStep = controller.selfCenter();
        double secondCenterStep = controller.selfCenter();
        double thirdCenterStep = controller.selfCenter();
        double centeredAngle = controller.selfCenter();

        assertAll(
                () -> assertEquals(180.0, firstCenterStep),
                () -> assertEquals(120.0, secondCenterStep),
                () -> assertEquals(60.0, thirdCenterStep),
                () -> assertEquals(0.0, centeredAngle));
    }

    @Test
    void givenLargeLowSpeedSteeringAngle_whenSpeedBecomesHigh_thenMovesTowardHighSpeedLimit() {
        SteeringController controller = new SteeringController();
        moveToAngle(controller, 300.0, 5);

        double reducedFirstStep = controller.update(450.0, 120.001);
        double reducedSecondStep = controller.update(450.0, 120.001);
        double reducedThirdStep = controller.update(450.0, 120.001);
        double limitedAngle = controller.update(450.0, 120.001);

        assertAll(
                () -> assertEquals(240.0, reducedFirstStep),
                () -> assertEquals(180.0, reducedSecondStep),
                () -> assertEquals(120.0, reducedThirdStep),
                () -> assertEquals(90.0, limitedAngle));
    }

    @Test
    void givenCurrentAngleNearNewTarget_whenUpdate_thenReachesTargetWithoutOvershoot() {
        SteeringController controller = new SteeringController();
        moveToAngle(controller, 240.0, 4);

        double angle = controller.update(250.0, 0.0);

        assertEquals(250.0, angle);
    }

    @Test
    void givenNaNAngle_whenUpdate_thenRejectsInput() {
        SteeringController controller = new SteeringController();

        assertThrows(IllegalArgumentException.class, () -> controller.update(Double.NaN, 0.0));
    }

    @Test
    void givenInfiniteAngle_whenUpdate_thenRejectsInput() {
        SteeringController controller = new SteeringController();

        assertAll(
                () -> assertThrows(IllegalArgumentException.class,
                        () -> controller.update(Double.POSITIVE_INFINITY, 0.0)),
                () -> assertThrows(IllegalArgumentException.class,
                        () -> controller.update(Double.NEGATIVE_INFINITY, 0.0)));
    }

    @Test
    void givenNegativeSpeed_whenUpdate_thenRejectsInput() {
        SteeringController controller = new SteeringController();

        assertThrows(IllegalArgumentException.class, () -> controller.update(0.0, -0.1));
    }

    @Test
    void givenNonFiniteSpeed_whenUpdate_thenRejectsInput() {
        SteeringController controller = new SteeringController();

        assertAll(
                () -> assertThrows(IllegalArgumentException.class,
                        () -> controller.update(0.0, Double.NaN)),
                () -> assertThrows(IllegalArgumentException.class,
                        () -> controller.update(0.0, Double.POSITIVE_INFINITY)));
    }

    @Test
    void givenSteeringState_whenUpdateRejectsInvalidInput_thenPreservesAngleForNextCommand() {
        SteeringController controller = new SteeringController();
        moveToAngle(controller, 240.0, 2);

        assertThrows(IllegalArgumentException.class, () -> controller.update(Double.NaN, 0.0));
        double centeredAngle = controller.selfCenter();

        assertEquals(60.0, centeredAngle);
    }

    private void moveToAngle(SteeringController controller, double requestedAngleDegrees, int updates) {
        for (int update = 0; update < updates; update++) {
            controller.update(requestedAngleDegrees, 0.0);
        }
    }

    private static Stream<Arguments> steeringUpdateCases() {
        return Stream.of(
                Arguments.of(45.0, 0.0, 1, 45.0),
                Arguments.of(500.0, 0.0, 8, 450.0),
                Arguments.of(-500.0, 121.0, 2, -90.0),
                Arguments.of(450.0, 120.0, 8, 450.0),
                Arguments.of(450.0, 120.001, 2, 90.0),
                Arguments.of(-450.0, 120.0, 8, -450.0),
                Arguments.of(-450.0, 120.001, 2, -90.0),
                Arguments.of(60.0, 150.0, 1, 60.0),
                Arguments.of(60.001, 0.0, 1, 60.0),
                Arguments.of(-60.001, 0.0, 1, -60.0),
                Arguments.of(91.0, 120.001, 2, 90.0),
                Arguments.of(89.0, 120.001, 2, 89.0),
                Arguments.of(-91.0, 120.001, 2, -90.0),
                Arguments.of(-89.0, 120.001, 2, -89.0));
    }
}
