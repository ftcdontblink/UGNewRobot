package org.firstinspires.ftc.teamcode.subsystems.subclasses;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class SideHood {
    public static final double MAX = 1;
    public static final double MIN = 0;
    public static final double MID = (MAX - MIN) / 2.0;

    InterpLUT lut;
    InterpLUT lutVelo;

    Servo sideHood;

    States states = States.AUTOMATIC;

    enum States {
        MANUAL,
        AUTOMATIC
    }

    public SideHood(HardwareMap hardwareMap) {
        sideHood = hardwareMap.get(Servo.class, "sidehood");

        sideHood.setPosition(MID);

        lut = new InterpLUT();
        lutVelo = new InterpLUT();

        lut.add(0, 0);
        lut.add(0, 0);
        lut.add(0, 0);
        lut.add(0, 0);
        lut.add(0, 0);

        lutVelo.add(-52, -0.2);
        lutVelo.add(0, 0);
        lutVelo.add(52, 0.2);
        lutVelo.add(0, 0);
        lutVelo.add(0, 0);

        lut.createLUT();
        lutVelo.createLUT();
    }

    public void update(double targetTheta, GamepadEx g1, GamepadEx g2) {
        // target = position
        // target = position + offset

        switch(states) {
            case MANUAL:
                sideHood.setPosition(MID);
                break;
            case AUTOMATIC:
                sideHood.setPosition(lut.get(targetTheta));
                break;
        }
    }

    public double getFirstDerivative(Vector2d target, Vector2d velocity, Pose2d estimate, double t) {
        double y = estimate.getX();
        double x = estimate.getY();

        double vy = velocity.getX();
        double vx = velocity.getY();

        double tx = target.getY();
        double ty = target.getX();

        double ans = ((vy/tx-x+vx*t) - ((vx*(ty-y-vy*t))/Math.pow((tx-x+vx*t), 2))) / (((Math.pow(((ty-y-vy*t)), 2))/Math.pow((tx-x+vx*t), 2)) + 1);

        return ans;
    }

    public double getSecondDerivative(Vector2d target, Vector2d velocity, Pose2d estimate, double t) {
        double y = estimate.getX();
        double x = estimate.getY();

        double vy = velocity.getX();
        double vx = velocity.getY();

        double tx = target.getY();
        double ty = target.getX();

        double ans = ((vy/tx-x+vx*t) - ((vx*(ty-y-vy*t))/Math.pow((tx-x+vx*t), 2))) / (((Math.pow(((ty-y-vy*t)), 2))/Math.pow((tx-x+vx*t), 2)) + 1);

        return ans;
    }
}
