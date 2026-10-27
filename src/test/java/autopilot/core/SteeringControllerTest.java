package autopilot.core;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.stream.Stream;

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
    void givenSteeredVehicle_whenSelfCenter_thenMovesTowardCenterByOneTick() {
        SteeringController controller = new SteeringController();
        controller.update(240.0, 0.0);

        double centeredAngle = controller.selfCenter();

        assertEquals(0.0, centeredAngle);
    }

    @Test
    void givenNonFiniteAngleOrInvalidSpeed_whenUpdate_thenRejectsInput() {
        SteeringController controller = new SteeringController();

        assertThrows(IllegalArgumentException.class,
                () -> controller.update(Double.NaN, 0.0));
        assertThrows(IllegalArgumentException.class,
                () -> controller.update(0.0, -1.0));
    }

    private static Stream<Arguments> steeringUpdateCases() {
        return Stream.of(
                Arguments.of(45.0, 0.0, 1, 45.0),
                Arguments.of(500.0, 0.0, 8, 450.0),
                Arguments.of(-500.0, 121.0, 2, -90.0));
    }
}
