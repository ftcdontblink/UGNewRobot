package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.vision.UGContourRingDetector;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Vision {
    UGContourRingPipeline.Height height;
    UGContourRingDetector detector;

    public Vision(HardwareMap hardwareMap, String name, Telemetry t) {
        detector = new UGContourRingDetector(hardwareMap,  name, t, true);
        detector.init();
    }

    public void update(Telemetry telemetry, LinearOpMode op) {
        while(!op.isStarted() && !op.isStopRequested()) {
            height = detector.getHeight();
            telemetry.addData("Height: ", height);
            telemetry.update();
        }
    }

    public UGContourRingPipeline.Height getHeight() {
        return height;
    }
}
