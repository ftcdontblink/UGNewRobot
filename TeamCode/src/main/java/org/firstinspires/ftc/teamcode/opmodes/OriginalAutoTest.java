package org.firstinspires.ftc.teamcode.opmodes;

import androidx.core.os.TraceKt;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Config
@Autonomous
public class OriginalAutoTest extends LinearOpMode {
    Robot robot;

    public static double velo = 4000;
    public static double angle = 0.23;

    public enum Paths {
        MOVE_TO_POWERSHOTS,
        SHOOT,
        DROP,
        STACK,
        STOP
    }

    Paths paths = Paths.MOVE_TO_POWERSHOTS;

    enum Color {
        RED,
        BLUE
    }

    Color color = Color.RED;

    public static double timing = 1000;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, gamepad1, gamepad2);
        ElapsedTime timer = new ElapsedTime();

        Trajectory goToShootBlue = robot.drive.trajectoryBuilder(new Pose2d(-63.5, -14, 0))
                .lineTo(new Vector2d(-12, 12))
                .build();

        Trajectory goToShootRed = robot.drive.trajectoryBuilder(new Pose2d(-63.5, -14, 0))
                .lineTo(new Vector2d(-12, -12))
                .build();

        while(!opModeIsActive()) {
            if(gamepad1.a) {
                color = Color.BLUE;
            }

            if(gamepad1.b) {
                color = Color.RED;
            }

            telemetry.addData("Color", color);
            telemetry.update();
        }

        waitForStart();

        if(color == Color.RED) {
            robot.drive.setPoseEstimate(new Pose2d(-63.5, -14, 0));
            angle = 0.23;
            robot.drive.followTrajectoryAsync(goToShootRed);
        } else {
            robot.drive.setPoseEstimate(new Pose2d(-63.5, 14, 0));
            robot.drive.followTrajectoryAsync(goToShootBlue);
            angle = 0.09;
        }

        robot.shooter.veloTimer.reset();

        while(opModeIsActive()) {
            switch(paths) {
                case MOVE_TO_POWERSHOTS:
                    Shooter.rpm = velo;
                    robot.mecanumDrive.sd.setPos(angle);
                    if(!robot.drive.isBusy()) {
                        timer.reset();
                        paths = Paths.SHOOT;
                    }
                    break;
                case SHOOT:
                    robot.shooter.shoot();

                    if(timer.milliseconds() > timing) {
                        robot.drive.followTrajectoryAsync(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).forward(25).build());
                        paths = Paths.STOP;
                    }
                    break;
                case STOP:
                    break;
            }

            robot.drive.update();
            robot.shooter.update();

            telemetry.addData("State", paths);
            telemetry.update();
        }
    }
}
