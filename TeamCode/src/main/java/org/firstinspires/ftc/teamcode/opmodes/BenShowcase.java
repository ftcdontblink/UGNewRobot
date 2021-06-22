package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class BenShowcase extends LinearOpMode {
    Pose2d startPose = new Pose2d(-63.2, -54, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(0, 0))
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> robot.wobbleGoal.clampOut())
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()-> robot.wobbleGoal.clampIn())
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> robot.wobbleGoal.clampOut())
                .waitSeconds(1) // total time for action
                .build();

        waitForStart();
        drive.followTrajectorySequenceAsync(sequence);
        while(opModeIsActive()) {
            drive.update();
        }
    }
}
