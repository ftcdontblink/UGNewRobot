package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * This opmode demonstrates how one would implement "align to point behavior" in teleop. You specify
 * a desired vector (x/y coordinate) via`targetPosition`. In the `ALIGN_TO_POINT` mode, the bot will
 * switch into field centric control and independently control its heading to align itself with the
 * specified `targetPosition`.
 * <p>
 * Press `a` to switch into alignment mode and `b` to switch back into standard teleop driving mode.
 * <p>
 * Note: We don't call drive.update() here because it has its own field drawing functions. We don't
 * want that to interfere with our graph so we just directly update localizer instead
 */
@Config
@TeleOp(group = "advanced")
public class TeleOpMain extends LinearOpMode {
    Robot robot;
    GamepadEx g1ex;
    GamepadEx g2ex;

    @Override
    public void runOpMode() throws InterruptedException {
        g1ex = new GamepadEx(gamepad1);
        g2ex = new GamepadEx(gamepad2);
        robot = new Robot(hardwareMap, g1ex, g2ex);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            robot.update(g1ex, g2ex);
        }
    }
}