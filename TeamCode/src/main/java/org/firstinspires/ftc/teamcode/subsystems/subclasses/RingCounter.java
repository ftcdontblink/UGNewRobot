package org.firstinspires.ftc.teamcode.subsystems.subclasses;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RingCounter {
    public static double enterDistance = 2;
    public static double exitDistance = 5;

    public static double minDistance = 4;
    public static double oneRingLow = 4;
    public static double oneRingHigh = 5;

    public static double twoRingLow = 4;
    public static double twoRingHigh = 5;

    public static double threeRingLow = 4;
    public static double threeRingHigh = 5;

    public static boolean isGoingThroughFront;
    public static boolean isGoingThroughBack;
    public static int ringCount;

    ElapsedTime time = new ElapsedTime();

    GamepadEx g1, g2;

    enum Sensor {
        HOPPER,
        FRONT,
        BACK
    }

    enum Count {
        GOING_THROUGH,
        STABLE,
    }

    Count count = Count.GOING_THROUGH;

    DistanceSensor hopperSensor;
    DistanceSensor frontIntake;
    DistanceSensor backIntake;

    public RingCounter(HardwareMap hardwareMap, boolean intake) {
        hopperSensor = hardwareMap.get(DistanceSensor.class, "hs");
        this.g1 = g1;
        this.g2 = g2;

        hopperSensor = hardwareMap.get(DistanceSensor.class, "hs");

        if(intake) {
            frontIntake = hardwareMap.get(DistanceSensor.class, "fis");
            backIntake = hardwareMap.get(DistanceSensor.class, "bis");
        }
    }

    public void update(Sensor one) {
        switch (count) {
            case STABLE:
                if(hopperSensor.getDistance(DistanceUnit.MM) < oneRingLow) {
                    ringCount = 0;
                }

                if(hopperSensor.getDistance(DistanceUnit.MM) < oneRingHigh
                && hopperSensor.getDistance(DistanceUnit.MM) > oneRingLow) {
                    ringCount = 1;
                }

                if(hopperSensor.getDistance(DistanceUnit.MM) < twoRingHigh
                        && hopperSensor.getDistance(DistanceUnit.MM) > twoRingLow) {
                    ringCount = 2;
                }

                if(hopperSensor.getDistance(DistanceUnit.MM) < threeRingHigh
                        && hopperSensor.getDistance(DistanceUnit.MM) > threeRingLow) {
                    ringCount = 3;
                }

                if(hopperSensor.getDistance(DistanceUnit.MM) < minDistance) {
                    count = Count.GOING_THROUGH;
                }
                break;
            case GOING_THROUGH:
                if(hopperSensor.getDistance(DistanceUnit.MM) < threeRingHigh) {
                    count = Count.STABLE;
                }
                break;
        }
    }

    public void update(Sensor one, Sensor two, GamepadEx g1, GamepadEx g2) {
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
