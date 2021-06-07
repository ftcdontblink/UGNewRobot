package org.firstinspires.ftc.teamcode.subsystems.subclasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class SideHood {
    public static final double MAX = 0.28;
    public static final double MIN = 0.08;
    public static final double MID = (MAX - MIN) / 2.0;

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

    public void update(double targetTheta, Gamepad g1, Gamepad g2) {
        switch(states) {
            case MANUAL:
                if(g1.left_stick_button)
                    states = States.AUTOMATIC;
                break;
            case AUTOMATIC:
                if(Math.toDegrees(targetTheta) > 180)
                    targetTheta -= Math.PI*2;

                if(Math.toDegrees(targetTheta) < -180)
                    targetTheta += Math.PI*2;

                sideHood.setPosition(thetaToPos(targetTheta));
                if(g1.right_stick_button) {
                    sideHood.setPosition(MID);
                    states = States.MANUAL;
                }
                break;
        }
    }

    public double thetaToPos(double theta) {
        double toDeg = Math.toDegrees(theta);
        double toPos = (-1.0/300.0)*toDeg;
        toPos = Range.clip(toPos, 0.08, 0.28);

        return toPos + OFFSET;
    }
}
