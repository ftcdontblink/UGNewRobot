package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Functions;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Config
@Autonomous
public class GoodBlueAuto extends LinearOpMode {
    com.arcrobotics.ftclib.vision.UGContourRingDetector detector;
    UGContourRingPipeline.Height height;

    Trajectory goToShootOut;
    Trajectory parkB;
    Trajectory dropWobbleBOut;

    Trajectory parkA;
    Trajectory dropWobbleAOut;

    Trajectory parkC;
    Trajectory dropWobbleCOut;

    FtcDashboard dashboard;
    Telemetry t;
    private Robot robot;
    private Functions functions;
    private Pose2d startPose = new Pose2d(-63.5, 14, 0);


    Servo sidehood;
    private double delay;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.drive.setShooter(robot.shooter, true);
        functions = new Functions(robot.drive, robot.shooter);

        dashboard = FtcDashboard.getInstance();
        t = dashboard.getTelemetry();

        robot.wobbleGoal.clampIn();

        goToShootOut = robot.drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-12, 16))
                .build();

        dropWobbleBOut = robot.drive.trajectoryBuilder(goToShootOut.end())
                .lineTo(new Vector2d(28, 16))
                .build();

        parkB = robot.drive.trajectoryBuilder(dropWobbleBOut.end())
                .lineTo(new Vector2d(5, 12))
                .build();

        dropWobbleAOut = robot.drive.trajectoryBuilder(goToShootOut.end())
                .lineTo(new Vector2d(5, 40))
                .build();

        parkA = robot.drive.trajectoryBuilder(dropWobbleAOut.end())
                .lineTo(new Vector2d(5, 16))
                .build();

        dropWobbleCOut = robot.drive.trajectoryBuilder(goToShootOut.end())
                .splineToConstantHeading(new Vector2d(45, 16), 0)
                .splineToConstantHeading(new Vector2d(45, 40), 0)
                .build();

        parkC = robot.drive.trajectoryBuilder(dropWobbleCOut.end())
                .splineToConstantHeading(new Vector2d(40, 16), 0)
                .splineToConstantHeading(new Vector2d(5, 16), 0)
                .build();

        sidehood = hardwareMap.get(Servo.class, "sidehood");

        detector = new com.arcrobotics.ftclib.vision.UGContourRingDetector(hardwareMap, "Webcam 2", telemetry, true);
        detector.init();
        height = detector.getHeight();

        delay = 0;

        while(!isStarted()) {
            height = detector.getHeight();
            if(gamepad1.dpad_up)
                delay+=0.001;
            if(gamepad1.dpad_down)
                delay-=0.001;

            telemetry.addData("Delay ", delay);
            telemetry.update();
        }

        waitForStart();

        robot.shooter.veloTimer.reset();
        robot.drive.setPoseEstimate(startPose);

        if(opModeIsActive()) {
            switch(height) {
                case ZERO:
                    aZoneOneWobble();
                    break;
                case ONE:
                    bZoneOneWobble();
                    break;
                case FOUR:
                    cZoneOneWobble();
                    break;
            }
        }
    }

    public void bZoneOneWobble() {
        startCondition();

        functions.sleepUpdate(delay);

        robot.drive.followTrajectory(dropWobbleBOut);

        dropInitialWobble();
        robot.drive.followTrajectory(parkB);
        stopCondition();
    }

    public void cZoneOneWobble() {
        startCondition();

        functions.sleepUpdate(delay);

        robot.drive.followTrajectory(dropWobbleCOut);

        dropInitialWobble();
        robot.drive.followTrajectory(parkC);
        stopCondition();
    }

    public void aZoneOneWobble() {
        startCondition();

        functions.sleepUpdate(delay);

        robot.drive.followTrajectory(dropWobbleAOut);

        dropInitialWobble();
        robot.drive.followTrajectory(parkA);
        stopCondition();
    }

    public void dropInitialWobble() {
        robot.wobbleGoal.clampOut();
        functions.sleepUpdate(600);
    }

    public void startCondition() {
        robot.intake.motor1.setPower(-1);
        robot.intake.motor2.setPower(-1);
        functions.sleepUpdate(200);
        robot.intake.motor1.setPower(0);
        robot.intake.motor2.setPower(0);
        Shooter.rpm = 3800;
        sidehood.setPosition(0.12);
        robot.drive.followTrajectory(goToShootOut);
        kick();
        kick();
        kick();
        kick();
        kick();
        kick();
        Shooter.rpm = 0;
    }

    public void stopCondition() {
        Shooter.rpm = 0;
        robot.intake.off();
    }

    public void kick() {
        robot.shooter.servo.setPosition(0.25);
        functions.sleepUpdate(150);
        robot.shooter.servo.setPosition(0.05);
        functions.sleepUpdate(150);
    }
}
