package org.firstinspires.ftc.teamcode.pathing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
public class Paths {
    Robot robot;
    SampleMecanumDrive drive;

    TrajectorySequence redFarA;
    TrajectorySequence redFarB;
    TrajectorySequence redFarC;

    public static Pose2d startPose = new Pose2d(-63.5, -14.5, 0);

    public enum Type {

    }



    public Paths(Robot robot, SampleMecanumDrive drive) {
        this.robot = robot;
        this.drive = drive;
    }

    public void createPaths() {
        redFarA = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-5, -14), 0)
                .splineToConstantHeading(new Vector2d(36, -19), 0)
                .lineTo(new Vector2d(36, -62))
                .lineTo(new Vector2d(36, -14))
                .lineTo(new Vector2d(5, -14))
                .build();

        redFarB = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-5, -14), 0)
                .splineToConstantHeading(new Vector2d(36, -19), 0)
                .lineTo(new Vector2d(5, -14))
                .build();

        redFarC = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-5, -14), 0)
                .splineToConstantHeading(new Vector2d(54, -19), 0)
                .lineTo(new Vector2d(5, -14))
                .build();
    }
}
