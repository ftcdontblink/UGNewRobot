package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends HardwareBase {
    DcMotor motor1;
    DcMotor motor2;

    public Intake(HardwareMap hardwareMap) {
        init(hardwareMap);
        super.init("Intake");
    }

    @Override
    public void init(HardwareMap map) {
        motor1 = map.get(DcMotor.class, "m1");
        motor2 = map.get(DcMotor.class, "m2");

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void update(Gamepad g1, Gamepad g2, Telemetry telemetry) {
        motor1.setPower(g2.right_trigger - g2.left_trigger);
        motor2.setPower(g2.right_trigger - g2.left_trigger);
    }
}
