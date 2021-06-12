package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opmodes.UGBasicHighGoalPipeline;
import org.firstinspires.ftc.teamcode.subsystems.subclasses.SideHood;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
public class Chassis extends HardwareBase {
    public static Vector2d targetPosition;
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);
    SampleMecanumDrive drive;
    Mode mode = Mode.NORMAL;
    public final double f = 453.46087317;
    ToggleButtonReader toggleA;

    public SideHood sd;

    public static double distance = 70;
    public static double b = 0.18;
    public static double kP = 0.001;
    public static double kS = 0.05;
    public static double tolerance = 25;



    Servo sidehood;
    UGBasicHighGoalPipeline pipeline;

    enum Mode {
        NORMAL,
        AIM_CAMERA,
        AIM_NORMAL
    }

    public enum Color {
        BLUE,
        RED
    }

    public Color color = Color.RED;

    public Chassis(SampleMecanumDrive drive, Gamepad g1) {
        this.drive = drive;
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        headingController.setInputBounds(-Math.PI, Math.PI);

        super.init("Chassis");
    }

    @Override
    public void init(HardwareMap map) {
        sidehood = map.get(Servo.class, "sidehood");
        pipeline = new UGBasicHighGoalPipeline();

        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName.class, "Webcam 2"),
                map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName()));

        camera.openCameraDevice();
        camera.setPipeline(pipeline);
        camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
        FtcDashboard.getInstance().startCameraStream((CameraStreamSource) camera, 30);
    }

    @Override
    public void update(Gamepad g1, Gamepad g2, Telemetry telemetry) {
        Double y = Math.copySign(Math.pow(-g1.left_stick_y, 1), -g1.left_stick_y);
        Double x = Math.copySign(Math.pow(-g1.left_stick_x, 1), -g1.left_stick_x);
        Double rotate = Math.copySign(Math.pow(-g1.right_stick_x, 1), -g1.right_stick_x);

        Pose2d driveDirection = new Pose2d(y, x, rotate);
        double relativeAngle = 0;

        if(color == Color.RED) {
            Rect redRect = pipeline.getRedRect();
            Point centerOfRedGoal = pipeline.getCenterofRect(redRect);

            telemetry.addData("Red goal position",

                    centerOfRedGoal.toString());
            telemetry.addData("Center: ", centerOfRedGoal);

            relativeAngle = (double) Math.atan((double) ((double) centerOfRedGoal.x - 360) / f);
        }

        if(color == Color.BLUE) {
            Rect blueRect = pipeline.getBlueRect();
            Point centerOfBlueGoal = pipeline.getCenterofRect(blueRect);

            telemetry.addData("Red goal position",

                    centerOfBlueGoal.toString());
            telemetry.addData("Center: ", centerOfBlueGoal);

            relativeAngle = (double) Math.atan((double) ((double) centerOfBlueGoal.x - 360) / f);
        }


        double a = 1.0/300.0;
        double pos = (a*Math.toDegrees(relativeAngle))+b;

        if(g1.a)
            sidehood.setPosition(a*Math.toDegrees(relativeAngle) + b);
        else
            sidehood.setPosition(b);

        if(g2.right_bumper) {
            color = Color.RED;
        }

        if(g2.left_bumper) {
            color = Color.BLUE;
        }

        telemetry.addData("Color", color);
        telemetry.addData("Mode", mode);

        drive.setWeightedDrivePower(driveDirection);
        drive.update();
    }
}
