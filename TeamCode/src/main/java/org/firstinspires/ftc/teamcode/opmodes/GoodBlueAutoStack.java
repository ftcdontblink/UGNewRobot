package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Functions;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.Arrays;

@Config
@Autonomous
public class GoodBlueAutoStack extends LinearOpMode {
    com.arcrobotics.ftclib.vision.UGContourRingDetector detector;
    UGContourRingPipeline.Height height;

    Trajectory goToShootOut;
    Trajectory parkB;
    Trajectory dropWobbleBOut;
    Trajectory goToStackB;
    Trajectory intakeStackB;
    Trajectory shootStackB;

    Trajectory goToStackC;
    Trajectory intakeStackC;
    Trajectory shootStackC;

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
    private Trajectory breakStackC;
    private Trajectory intakeStack2C;


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

        goToStackB = robot.drive.trajectoryBuilder(dropWobbleBOut.end())
                .splineToConstantHeading(new Vector2d(20, 16), 0)
                .splineToConstantHeading(new Vector2d(0, 32), 0)
                .splineToConstantHeading(new Vector2d(-14, 32), 0)
                .build();

        intakeStackB = robot.drive.trajectoryBuilder(goToStackB.end())
                .back(25)
                .addDisplacementMarker(() -> sidehood.setPosition(0.19))
                .build();

        shootStackB = robot.drive.trajectoryBuilder(intakeStackB.end())
                .lineTo(new Vector2d(-12, 34))
                .build();

        parkB = robot.drive.trajectoryBuilder(shootStackB.end())
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
                .splineToConstantHeading(new Vector2d(47, 40), 0)
                .build();

        goToStackC = robot.drive.trajectoryBuilder(dropWobbleCOut.end())
                .splineToConstantHeading(new Vector2d(-14, 24), 0)
                .splineToConstantHeading(new Vector2d(-20, 24), 0)
                .splineToConstantHeading(new Vector2d(-14, 24), 0)
                .build();

//        breakStackC = robot.drive.trajectoryBuilder(goToStackC.end())
//                .splineToConstantHeading(new Vector2d(-20,24), 0)
//                .splineToConstantHeading(new Vector2d(-14,32), 0)
//                .build();

        intakeStackC = robot.drive.trajectoryBuilder(goToStackC.end())
                .back(10,
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(15.0, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        intakeStack2C = robot.drive.trajectoryBuilder(intakeStackC.end())
                .back(30,
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(15.0, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        shootStackC = robot.drive.trajectoryBuilder(intakeStack2C.end())
                .lineTo(new Vector2d(-12, 34))
                .build();

        parkC = robot.drive.trajectoryBuilder(shootStackC.end())
                .lineTo(new Vector2d(5, 12))
                .build();

        sidehood = hardwareMap.get(Servo.class, "sidehood");

        detector = new com.arcrobotics.ftclib.vision.UGContourRingDetector(hardwareMap, "Webcam 2", telemetry, true);
        detector.init();
        height = detector.getHeight();

        while(!isStarted()) {
            height = detector.getHeight();
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

        robot.drive.followTrajectory(dropWobbleBOut);

        dropInitialWobble();

        robot.drive.followTrajectory(goToStackB);

        robot.intake.on();

        robot.drive.followTrajectory(intakeStackB);

        functions.sleepUpdate(5000);

        robot.intake.off();

        functions.sleepUpdate(1000);

        Shooter.rpm = 4000;

        robot.drive.followTrajectory(shootStackB);

        kick();
        kick();

        Shooter.rpm = 0;

        robot.intake.off();

        robot.drive.followTrajectory(parkB);
        stopCondition();
    }

    public void cZoneOneWobble() {
        startCondition();

        robot.drive.followTrajectory(dropWobbleCOut);

        dropInitialWobble();

        robot.drive.followTrajectory(goToStackC);
//        robot.drive.followTrajectory(breakStackC);
        robot.intake.on();
        robot.drive.followTrajectory(intakeStackC);

        functions.sleepUpdate(2000);

        robot.intake.off();

        robot.shooter.on(3800);
        functions.sleepUpdate(1000);

        sidehood.setPosition(0.2);
        kick();
        kick();
        kick();
        kick();
        robot.shooter.off();
        robot.intake.on();
        robot.drive.followTrajectory(intakeStack2C);
        functions.sleepUpdate(2000);
        robot.drive.followTrajectory(shootStackC);
        robot.intake.off();
        functions.sleepUpdate(1000);
        robot.shooter.on(4000);
        sidehood.setPosition(0.18);
        kick();
        kick();
        kick();
        kick();
        robot.shooter.off();
        robot.drive.followTrajectory(parkC);
        stopCondition();
    }

    public void aZoneOneWobble() {
        startCondition();

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
        Shooter.rpm = 4000;
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
