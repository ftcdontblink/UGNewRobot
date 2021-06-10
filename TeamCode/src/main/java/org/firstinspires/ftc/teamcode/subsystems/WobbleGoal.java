package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WobbleGoal extends HardwareBase {
    public Servo turret;
    public Servo arm1;
    public Servo arm2;
    public Servo claw;

    public static double armPos1;
    public static double armPos2;
    public static double clawPos;

    public WobbleGoal(HardwareMap hardwareMap) {
        super.init("Wobble Goal");
        init(hardwareMap);
    }

    public enum WOBBLE_STATE {
        WOBBLE_RIGHT,
        WOBBLE_UP,
        WOBBLE_DEPOSIT,
        WOBBLE_LEFT
    }

    WOBBLE_STATE state = WOBBLE_STATE.WOBBLE_RIGHT;
    ElapsedTime time = new ElapsedTime();

    @Override
    public void init(HardwareMap map) {
        turret = map.get(Servo.class, "turret");
        arm1 = map.get(Servo.class, "a1");
        arm2 = map.get(Servo.class, "a2");
        claw = map.get(Servo.class, "claw");
    }

    @Override
    public void update(Gamepad g1, Gamepad gamepad, Telemetry telemetry) {
        turret.setPosition(0.4);

        switch (state) {
            case WOBBLE_RIGHT:
                armPos1 = 0.4;
                armPos2 = 0.6;
                clawPos = 0.775;


                if (gamepad.dpad_left) state = WOBBLE_STATE.WOBBLE_LEFT;
                if (gamepad.b) state = WOBBLE_STATE.WOBBLE_DEPOSIT;
                if (gamepad.dpad_up) {
                    state = WOBBLE_STATE.WOBBLE_UP;
                    time.reset();
                }
                break;
            case WOBBLE_UP:
                clawPos = 0.775;
                if(time.milliseconds() > 750) {
                    armPos1 = 0.35;
                    armPos2 = 0.65;
                }
                if (gamepad.dpad_right) state = WOBBLE_STATE.WOBBLE_RIGHT;
                if (gamepad.dpad_left) state = WOBBLE_STATE.WOBBLE_LEFT;
                if (gamepad.b) state = WOBBLE_STATE.WOBBLE_DEPOSIT;

                break;
            case WOBBLE_LEFT:
                armPos1 = 0;
                armPos2 = 1;
                clawPos = 0.5;
                if (gamepad.dpad_right) state = WOBBLE_STATE.WOBBLE_RIGHT;
                if (gamepad.b) state = WOBBLE_STATE.WOBBLE_DEPOSIT;
                if (gamepad.dpad_up) {
                    state = WOBBLE_STATE.WOBBLE_UP;
                    time.reset();
                }
                break;
            case WOBBLE_DEPOSIT:
                clawPos = 0.5;

                if (gamepad.dpad_left) state = WOBBLE_STATE.WOBBLE_LEFT;

                if (gamepad.dpad_up) {
                    state = WOBBLE_STATE.WOBBLE_UP;
                    time.reset();
                }
                break;
        }

        arm1.setPosition(armPos1);
        arm2.setPosition(armPos2);
        claw.setPosition(clawPos);
    }
}
