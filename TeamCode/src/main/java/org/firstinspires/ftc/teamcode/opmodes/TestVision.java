package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class TestVision extends LinearOpMode {
    public final double f = 453.46087317;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo sidehood = hardwareMap.get(Servo.class, "sidehood");
        UGBasicHighGoalPipeline pipeline = new UGBasicHighGoalPipeline();

        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"),
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));

        camera.openCameraDevice();
        camera.setPipeline(pipeline);
        camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
        FtcDashboard.getInstance().startCameraStream((CameraStreamSource) camera, 30);

        waitForStart();

        while(opModeIsActive()) {
            // Red Goal
            Rect redRect = pipeline.getRedRect();
                Point centerOfRedGoal = pipeline.getCenterofRect(redRect);

                telemetry.addData("Red goal position",

                        centerOfRedGoal.toString());
                telemetry.addData("Center: ", centerOfRedGoal);

                double relativeAngle = (double)Math.atan((double)((double)centerOfRedGoal.x - 360) / f);
                double a = 1.0/300.0;
                double b = 0.18;
                double pos = (a*Math.toDegrees(relativeAngle))+b;

                sidehood.setPosition(pos);
                telemetry.addData("Relative", relativeAngle);
                telemetry.addData("Relative", Math.toDegrees(relativeAngle));
                telemetry.addData("Pos", pos);
                telemetry.update();


        }
    }
}
