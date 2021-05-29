package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
        motor1 = map.get(DcMotor.class, "motor1");
        motor2 = map.get(DcMotor.class, "motor2");

        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void update(GamepadEx g1, GamepadEx g2, Telemetry telemetry) {
        motor1.setPower(g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        motor2.setPower(g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
    }
}
