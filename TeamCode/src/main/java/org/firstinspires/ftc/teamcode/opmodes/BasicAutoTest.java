package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.subclasses.Functions;

@Config
@Autonomous
public class BasicAutoTest extends LinearOpMode {
    Robot robot;
    Functions functions;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, gamepad1, gamepad2);
        robot.drive.setShooter(robot.shooter, true);
        functions = new Functions(robot.drive, robot.shooter);

        ElapsedTime timer = new ElapsedTime();

        Trajectory goToShootBlue = robot.drive.trajectoryBuilder(new Pose2d(-63.5, -14, 0))
                .lineTo(new Vector2d(-12, 12))
                .build();

        Trajectory goToShootRed = robot.drive.trajectoryBuilder(new Pose2d(-63.5, -14, 0))
                .lineTo(new Vector2d(-12, -12))
                .build();

        Trajectory goToWobble = robot.drive.trajectoryBuilder(goToShootRed.end())
                .lineTo(new Vector2d(42-10, -20))
                .build();

        Trajectory stack = robot.drive.trajectoryBuilder(goToWobble.end())
                .splineToConstantHeading(new Vector2d(-13, -16), 0)
                .splineToConstantHeading(new Vector2d(-13, -38), 0)
                .build();

        Trajectory driveBack = robot.drive.trajectoryBuilder(stack.end())
                .back(20)
                .build();

        Trajectory goToShoot = robot.drive.trajectoryBuilder(driveBack.end())
                .lineTo(new Vector2d(-12, -12))
                .build();

        robot.wobbleGoal.turret.setPosition(0.4);
        robot.wobbleGoal.claw.setPosition(0.775);
        robot.wobbleGoal.arm2.setPosition(0.6);
        robot.wobbleGoal.arm1.setPosition(0.4);

        waitForStart();

        robot.shooter.veloTimer.reset();
        robot.drive.setPoseEstimate(new Pose2d(-63.5 , -14, 0));

        if(opModeIsActive()) {
            Shooter.rpm = 3400;
            robot.mecanumDrive.sd.setPos(0.18);
            robot.drive.followTrajectory(goToShoot);
            kick();
            functions.sleepUpdate(200);
            robot.mecanumDrive.sd.setPos(0.16);
            kick();
            functions.sleepUpdate(200);
            robot.mecanumDrive.sd.setPos(0.14);
            kick();

            telemetry.update();
        }
    }

    public void kick() {
        robot.shooter.servo.setPosition(0.2);
        functions.sleepUpdate(100);
        robot.shooter.servo.setPosition(0.05);
        functions.sleepUpdate(100);
    }
}
