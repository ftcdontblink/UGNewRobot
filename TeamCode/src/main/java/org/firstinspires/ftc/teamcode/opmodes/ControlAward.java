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

@Config
@Autonomous
public class ControlAward extends LinearOpMode {
    Robot robot;

    public static double velo = 4000;
    public static double angle = 0.23;

    public enum Paths {
        MOVE_TO_POWERSHOTS,
        SHOOT,
        DROP,
        DROP_ITSELF,
        STACK,
        INTAKE,
        GOBACK,
        SHOOT2,
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
            angle = 0.235;
            robot.drive.followTrajectoryAsync(goToShootRed);
        } else {
            robot.drive.setPoseEstimate(new Pose2d(-63.5, 14, 0));
            robot.drive.followTrajectoryAsync(goToShootBlue);
            angle = 0.09;
        }

        robot.shooter.veloTimer.reset();

        while(opModeIsActive()) {
            robot.wobbleGoal.turret.setPosition(0);

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
                        Shooter.rpm = 0;
                        robot.drive.followTrajectoryAsync(goToWobble);
                        paths = Paths.DROP;
                    }
                    break;
                case DROP:
                    robot.wobbleGoal.arm2.setPosition(1);
                    robot.wobbleGoal.arm1.setPosition(0);

                    if(!robot.drive.isBusy()) {
                        robot.wobbleGoal.claw.setPosition(0.5);
                        timer.reset();
                        paths = Paths.DROP_ITSELF;
                    }
                    break;
                case DROP_ITSELF:
                    if(timer.seconds() == 0.5) {
                        robot.wobbleGoal.arm2.setPosition(0.65);
                        robot.wobbleGoal.arm1.setPosition(0.35);
                    }

                    if(timer.seconds() > 1) {
                        robot.drive.followTrajectoryAsync(stack);
                        paths = Paths.STACK;
                    }
                    break;
                case STACK:
                    if(!robot.drive.isBusy()) {
                        robot.intake.on();
                        robot.drive.followTrajectoryAsync(driveBack);
                        paths = Paths.INTAKE;
                    }
                    break;
                case INTAKE:
                    if(!robot.drive.isBusy()) {
                        robot.drive.followTrajectoryAsync(goToShoot);
                        paths = Paths.GOBACK;
                    }
                    break;
                case GOBACK:
                    robot.mecanumDrive.sd.setPos(0.22);
                    Shooter.rpm = 4100;
                    if(!robot.drive.isBusy()) {
                        timer.reset();
                        paths = Paths.SHOOT2;
                    }
                    break;
                case SHOOT2:
                    robot.shooter.shoot();

                    if(timer.milliseconds() > timing) {
                        robot.wobbleGoal.turret.setPosition(0.1);
                        Shooter.rpm = 0;
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
