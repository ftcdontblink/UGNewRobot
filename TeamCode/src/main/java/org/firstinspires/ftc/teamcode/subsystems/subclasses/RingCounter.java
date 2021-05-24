package org.firstinspires.ftc.teamcode.subsystems.subclasses;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RingCounter {
    public static double enterDistance = 2;
    public static double exitDistance = 5;

    public static boolean isGoingThroughFront;
    public static boolean isGoingThroughBack;
    public static int ringCount;

    GamepadEx g1, g2;

    enum Sensor {
        HOPPER,
        FRONT,
        BACK
    }

    DistanceSensor hopperSensor;
    DistanceSensor frontIntake;
    DistanceSensor backIntake;

    public RingCounter(HardwareMap hardwareMap, GamepadEx g1, GamepadEx g2) {
        hopperSensor = hardwareMap.get(DistanceSensor.class, "hs");
        this.g1 = g1;
        this.g2 = g2;
    }

    public RingCounter(HardwareMap hardwareMap, boolean intake) {
        hopperSensor = hardwareMap.get(DistanceSensor.class, "hs");

        if(intake) {
            frontIntake = hardwareMap.get(DistanceSensor.class, "fis");
            backIntake = hardwareMap.get(DistanceSensor.class, "bis");
        }
    }

    public void update(Sensor one) {
        if(!isGoingThroughFront) {
            if(frontIntake.getDistance(DistanceUnit.INCH) < enterDistance) {
                isGoingThroughFront = true;
            }
        }

        if(isGoingThroughFront) {
            if(frontIntake.getDistance(DistanceUnit.INCH) > exitDistance) {
                isGoingThroughFront = false;

                if(g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0) ringCount++;
                if(g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0) ringCount--;
            }
        }
    }

    public void update(Sensor one, Sensor two) {
        if(!isGoingThroughFront) {
            if(frontIntake.getDistance(DistanceUnit.INCH) < enterDistance) {
                isGoingThroughFront = true;
            }
        }

        if(isGoingThroughFront) {
            if(frontIntake.getDistance(DistanceUnit.INCH) > exitDistance) {
                isGoingThroughFront = false;

                if(g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0) ringCount++;
                if(g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0) ringCount--;
            }
        }

        // ------------------------------------------------------------------------------

        if(!isGoingThroughBack) {
            if(backIntake.getDistance(DistanceUnit.INCH) < enterDistance) {
                isGoingThroughBack = true;
            }
        }

        if(isGoingThroughBack) {
            if(backIntake.getDistance(DistanceUnit.INCH) > exitDistance) {
                isGoingThroughBack = false;

                if(g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0) ringCount++;
                if(g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0) ringCount--;
            }
        }
    }
}
