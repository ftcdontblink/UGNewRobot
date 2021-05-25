package org.firstinspires.ftc.teamcode.subsystems.subclasses;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class HighGoalPipeline extends OpenCvPipeline {

    private Mat maskMat = new Mat();
    private Mat outputMat = new Mat();

    public Scalar redLower = new Scalar(0, 100, 138);
    public Scalar redUpper = new Scalar(8, 209, 255);

    public Scalar blueLower = new Scalar(110, 73, 19);
    public Scalar blueUpper = new Scalar(113, 199, 205);

    public Scalar redDrawingColor = new Scalar(255, 20, 20);
    public Scalar blueDrawingColor = new Scalar(20, 20, 255);

    public double ratioMin = 1.20;
    public double ratioMax = 1.40;

    public int areaMin = 800;
    public int areaMax = 2000;

    private Goal redGoal = new Goal();
    private Goal blueGoal = new Goal();

    public double center;

    Telemetry telemetry;

    public HighGoalPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        redGoal.process(input, redLower, redUpper);
        blueGoal.process(input, blueLower, blueUpper);

        redGoal.addTo(blueGoal, maskMat);

        outputMat.release();
        Core.bitwise_and(input, input, outputMat, maskMat);

        Imgproc.drawContours(outputMat, redGoal.contours, -1, redDrawingColor, 1);
        Imgproc.drawContours(outputMat, blueGoal.contours, -1, blueDrawingColor, 1);

        if(redGoal.highGoalRect != null) {
            Imgproc.rectangle(outputMat, redGoal.highGoalRect, redDrawingColor, 2);
            telemetry.addData("Red goal", "Detected");
            telemetry.addData("Red goal A", redGoal.highGoalRect.area());
        }

        center = redGoal.highGoalRect.area();

        if(blueGoal.highGoalRect != null) {
            Imgproc.rectangle(outputMat, blueGoal.highGoalRect, blueDrawingColor, 2);
            telemetry.addData("Blue goal", "Detected");
            telemetry.addData("Blue goal A", blueGoal.highGoalRect.area());
        }

        telemetry.update();

        return outputMat;
    }

    public double getCenter() {
        return center;
    }

    private class Goal {

        private Mat hsvMat  = new Mat();
        private Mat binaryMat = new Mat();

        private Mat hierarchyMat = new Mat();

        private ArrayList<MatOfPoint> contours = new ArrayList<>();
        private Rect highGoalRect = null;

        public void process(Mat input, Scalar lower, Scalar upper) {
            highGoalRect = null;

            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsvMat, lower, upper, binaryMat);

            contours.clear();
            Imgproc.findContours(
                    binaryMat, contours, hierarchyMat,
                    Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE
            );

            hierarchyMat.release();

            for(MatOfPoint contour : contours) {
                Rect rect = Imgproc.boundingRect(contour);

                if(!isAcceptableRect(rect))
                    continue;

                if(highGoalRect == null) {
                    highGoalRect = rect;
                }

                if(rect.area() > highGoalRect.area()) {
                    highGoalRect = rect;
                }
            }
        }

        private boolean isAcceptableRect(Rect rect) {
            double ratio = (double)rect.width / (double)rect.height;

            return ratio >= ratioMin && ratio <= ratioMax &&
                    rect.area() >= areaMin && rect.area() <= areaMax;
        }

        public void addTo(Goal other, Mat output) {
            output.release();
            Core.bitwise_or(binaryMat, other.binaryMat, output);
        }

    }

}