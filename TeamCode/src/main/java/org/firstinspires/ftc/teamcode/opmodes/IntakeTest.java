package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class IntakeTest extends LinearOpMode {
    Robot robot;
    GamepadEx g1;
    GamepadEx g2;

    @Override
    public void runOpMode() throws InterruptedException {
        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);
        robot = new Robot(hardwareMap, g1, g2);

        waitForStart();

        while(opModeIsActive()) {
            robot.update(g1, g2);
        }
    }
}
