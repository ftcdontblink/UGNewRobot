package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleGoal extends HardwareBase {
    Servo turret;
    Servo arm1;
    Servo arm2;
    Servo claw;

    public WobbleGoal(HardwareMap hardwareMap) {
        super.init("Wobble Goal");
    }

    @Override
    public void init(HardwareMap map) {
        turret = map.get(Servo.class, "turret");
        arm1 = map.get(Servo.class, "a1");
        arm2 = map.get(Servo.class, "a2");
        claw = map.get(Servo.class, "claw");
    }

    @Override
    public void update(GamepadEx g1, GamepadEx g2) {

    }
}
