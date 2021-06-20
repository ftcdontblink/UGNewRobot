package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.vision.UGContourRingDetector;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Functions;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Config
@Autonomous
public class FinalsAutoBlue extends LinearOpMode {
    Robot robot;
    Functions functions;
    UGContourRingDetector detector;
    UGContourRingPipeline.Height height;

    Pose2d startPose = new Pose2d(-63.5, 14, 0);

    Trajectory goToPowershots;

    Servo sidehood;
    public static double angle1 = 0.145;
    public static double angle2 = 0.165;
    public static double angle3 = 0.18;
    public static double velocity = 3100;
    public static double sleep = 250;
    private Trajectory goToBounceBacks;
    private Trajectory goToB;
    private Trajectory goToA;
    private Trajectory goToB2;
    private Trajectory goToC;
    private Trajectory goToShootB;
    private Trajectory goToShootC1;
    private Trajectory goToShootC2;
    private Trajectory backA;
    private Trajectory goToShootA;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.drive.setShooter(robot.shooter, true);
        functions = new Functions(robot.drive, robot.shooter);


        robot.wobbleGoal.clampIn();

        goToPowershots = robot.drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-12, 16, Math.toRadians(-17)), Math.toRadians(0))
                .build();

        goToBounceBacks = robot.drive.trajectoryBuilder(goToPowershots.end())
                .splineToLinearHeading(new Pose2d(49, 12, 0), 0)
                .build();

        goToB = robot.drive.trajectoryBuilder(goToPowershots.end())
                .lineTo(new Vector2d(24, -16))
                .addDisplacementMarker(() -> robot.wobbleGoal.clampOut())
                .build();

        goToShootB = robot.drive.trajectoryBuilder(goToB.end())
                .lineTo(new Vector2d(-12, -16))
                .addDisplacementMarker(() -> robot.wobbleGoal.clampOut())
                .build();

        goToC = robot.drive.trajectoryBuilder(goToBounceBacks.end())
                .lineTo(new Vector2d(49, -40))
                .addDisplacementMarker(() -> robot.wobbleGoal.clampOut())
                .build();

        goToShootC1 = robot.drive.trajectoryBuilder(goToC.end())
                .lineTo(new Vector2d(46, -16))
                .build();

        goToShootC2 = robot.drive.trajectoryBuilder(goToShootC1.end())
                .lineTo(new Vector2d(-12, -16))
                .addDisplacementMarker(() -> robot.wobbleGoal.clampOut())
                .build();

        goToA = robot.drive.trajectoryBuilder(goToPowershots.end())
                .splineToLinearHeading(new Pose2d(30, -52, Math.toRadians(-90)), Math.toRadians(-90))
                .addDisplacementMarker(() -> robot.wobbleGoal.clampOut())
                .build();

        backA = robot.drive.trajectoryBuilder(goToA.end())
                .lineTo(new Vector2d(32, -16))
                .build();

        goToShootA = robot.drive.trajectoryBuilder(goToA.end())
                .lineTo(new Vector2d(-12, -16))
                .build();

        sidehood = hardwareMap.get(Servo.class, "sidehood");

        detector = new UGContourRingDetector(hardwareMap, "Webcam 2", telemetry, true);
        detector.init();
        height = detector.getHeight();

        while(!opModeIsActive()) {
            height = detector.getHeight();
        }

        waitForStart();

        robot.drive.setPoseEstimate(startPose);

        if(opModeIsActive()) {
            switch(height) {
                case ZERO:
                    aZone();
                    break;
                case ONE:
                    bZone();
                    break;
                case FOUR:
                    cZone();
                    break;
            }
        }
    }

    public void startConditions() {
        sidehood.setPosition(0);
        Shooter.rpm = velocity;

        robot.drive.follower = new HolonomicPIDVAFollower(SampleMecanumDrive.TRANSLATIONAL_PID, SampleMecanumDrive.TRANSLATIONAL_PID, SampleMecanumDrive.HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(0.5)), 3);

        robot.drive.followTrajectory(goToPowershots);
        functions.sleepUpdate(sleep);
        kick();
//        sidehood.setPosition(angle2);
        robot.drive.turn(Math.toRadians(-6));
        functions.sleepUpdate(sleep);
        kick();
//        sidehood.setPosition(angle3);
        robot.drive.turn(Math.toRadians(-6));
        functions.sleepUpdate(sleep);
        kick();
        kick();
        robot.drive.turn(Math.toRadians(17+6+6));
        Shooter.rpm = 0;
        robot.intake.motor1.setPower(-1);
        robot.intake.motor2.setPower(-1);
        functions.sleepUpdate(100);
        robot.intake.motor1.setPower(0);
        robot.intake.motor2.setPower(0);
        robot.intake.on();
        robot.drive.follower = new HolonomicPIDVAFollower(SampleMecanumDrive.TRANSLATIONAL_PID, SampleMecanumDrive.TRANSLATIONAL_PID, SampleMecanumDrive.HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(2.5)), 1);
        robot.drive.followTrajectory(goToBounceBacks);
    }

    public void kick() {
        robot.shooter.servo.setPosition(0.25);
        functions.sleepUpdate(150);
        robot.shooter.servo.setPosition(0.05);
        functions.sleepUpdate(150);
    }
    public void aZone() {
        startConditions();
//        robot.drive.followTrajectory(goToA);
//        robot.drive.followTrajectory(backA);
//        robot.drive.turn(Math.toRadians(90));
//        robot.drive.followTrajectory(goToShootA);
//        shootBouncebacks();
    }

    public void bZone() {
        startConditions();
        robot.drive.followTrajectory(goToB);
        robot.drive.followTrajectory(goToShootB);
        shootBouncebacks();
    }

    public void cZone() {
        startConditions();
        robot.drive.followTrajectory(goToC);
        robot.drive.followTrajectory(goToShootC1);
        robot.drive.followTrajectory(goToShootC2);
        shootBouncebacks();
    }

    public void shootBouncebacks() {
        robot.intake.off();
        Shooter.rpm = 3800;
        sidehood.setPosition(0.26);
        functions.sleepUpdate(800);
        kick();
        kick();
        kick();
        kick();
        kick();
        Shooter.rpm = 3800;
        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(new Pose2d(-12, 16, 0)).forward(20).build());
    }
}
