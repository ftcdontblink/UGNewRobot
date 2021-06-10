package org.firstinspires.ftc.teamcode.subsystems.subclasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class SideHood {
    public static double MAX = 0.28;
    public static double MIN = 0.08;
    public static double MID = (MAX - MIN) / 2.0;

    Servo sideHood;

    States states = States.AUTOMATIC;
    public static double OFFSET = MID;

    enum States {
        MANUAL,
        AUTOMATIC
    }

    public SideHood(HardwareMap hardwareMap) {
        sideHood = hardwareMap.get(Servo.class, "sidehood");

        sideHood.setPosition(MID);
    }

    public void update(double position, Gamepad g1) {
        switch(states) {
            case MANUAL:
                sideHood.setPosition(0.15);
                if(g1.left_stick_button)
                    states = States.AUTOMATIC;
                break;
            case AUTOMATIC:
                sideHood.setPosition(position);
                if(g1.right_stick_button) {
                    sideHood.setPosition(MID);
                    states = States.MANUAL;
                }
                break;
        }
    }

    public void setPos(double x) {
        sideHood.setPosition(x);
    }

    public double thetaToPos(double theta) {
        double toDeg = Math.toDegrees(theta);
        double toPos = (-1.0/300.0)*toDeg;
        toPos = Range.clip(toPos, 0.08, 0.28);

        return toPos + OFFSET;
    }
}
