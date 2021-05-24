package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

@Autonomous
@Config
public class AutoTest extends LinearOpMode {
    Robot robot;
    Vision vision;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        vision = new Vision(hardwareMap, "camera", telemetry);

        vision.update(telemetry, this);
        waitForStart();

        TrajectorySequence builder;

        builder = robot.drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .splineToConstantHeading(new Vector2d(-5, -14), 0)
                .splineTo(new Vector2d(36-20, -8), 0)
                .splineTo(new Vector2d(63.5-24, -58), 3*Math.PI/2)
                .turn(Math.PI/2)
                .forward(20)
                .turn(Math.PI/2)
                .forward(50)
                .splineToLinearHeading(new Pose2d(-10, -36 ,0), Math.PI)

                .lineTo(new Vector2d(-32, -36), new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(Math.toRadians(180)),
                                        new MecanumVelocityConstraint(7.5, 15)
                                )),
                        new ProfileAccelerationConstraint(52.48291908330528)
                )
                .waitSeconds(1)
                .lineTo(new Vector2d(53.5, -46))
                .waitSeconds(1)
                .lineTo(new Vector2d(-5, -36))
                .build();

        while(opModeIsActive()) {
            robot.drive.followTrajectorySequenceAsync(builder);
        }
    }
}
