package autopilot.core;

import autopilot.exception.IllegalGearChangeException;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

import static org.junit.jupiter.api.Assertions.assertEquals;
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
    void givenVehicleAtReverseEntryLimit_whenChangeToReverse_thenRejectsTransition() {
        sensedSpeedKmh = 5.0;

        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(GearBox.Gear.R));
    }

    @Test
    void givenMovingVehicleInReverse_whenChangeToDrive_thenRejectsTransition() {
        gearBox.changeTo(GearBox.Gear.R);
        sensedSpeedKmh = 1.0;

        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(GearBox.Gear.D));
    }

    @Test
    void givenInvalidSensorReading_whenChangeToReverse_thenRejectsTransition() {
        sensedSpeedKmh = Double.NaN;

        assertThrows(IllegalArgumentException.class, () -> gearBox.changeTo(GearBox.Gear.R));
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
