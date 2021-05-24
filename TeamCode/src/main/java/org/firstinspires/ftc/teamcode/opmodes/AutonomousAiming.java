package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

@Config
public class AutonomousAiming {
    public enum State {
        OFF,
        AIMING_HIGH_GOAL,
        AIMING_POWERSHOT,
        SHOOTING_HIGH_GOAL,
        SHOOTING_POWERSHOT,
        WAITING_POWERSHOT
    }

    private State state = State.OFF;

    private final OpenCvWebcam webcam;
    private SampleMecanumDrive drive;

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime pidTimer = new ElapsedTime();

    private int currentPowershot = 0;
    private double lastX = Double.MAX_VALUE;
    private MovingStatistics lastXs = new MovingStatistics(5);
    private double lastDelta = 0.0;


    public static double GAIN = 1.0;
    public static double EXPOSURE = 0.12;

    private AimingPipeline aiming = new AimingPipeline();


    public AutonomousAiming(HardwareMap hardwareMap, SampleMecanumDrive drive) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(aiming);
        webcam.openCameraDeviceAsync(() -> {
            FtcDashboard.getInstance().startCameraStream(webcam, 30);
            webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

            GainControl gainControl = webcam.getGainControl();
            gainControl.setGain((int) Range.scale(GAIN, 0, 1, gainControl.getMinGain(), gainControl.getMaxGain()));
            ExposureControl exposureControl = webcam.getExposureControl();
            exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureControl.setExposure((long) Range.scale(EXPOSURE, 0, 1,
                    exposureControl.getMinExposure(TimeUnit.NANOSECONDS),
                    exposureControl.getMaxExposure(TimeUnit.NANOSECONDS)), TimeUnit.NANOSECONDS);
        });

        this.drive = drive;
    }

    public double update() {
        GainControl gainControl = webcam.getGainControl();
        gainControl.setGain((int) Range.scale(GAIN, 0, 1, gainControl.getMinGain(), gainControl.getMaxGain()));
        ExposureControl exposureControl = webcam.getExposureControl();
        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure((long) Range.scale(EXPOSURE, 0, 1,
                exposureControl.getMinExposure(TimeUnit.NANOSECONDS),
                exposureControl.getMaxExposure(TimeUnit.NANOSECONDS)), TimeUnit.NANOSECONDS);

        return aiming.getGoalCenterX();
    }
}
