package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Chassis extends HardwareBase {
    public static Vector2d targetPosition = new Vector2d(72, -36);
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);
    SampleMecanumDrive drive;
    Mode mode = Mode.NORMAL;
    ToggleButtonReader toggleA;

    public static double distance = 70;

    enum Mode {
        NORMAL,
        AIM,
        AIM_NORMAL
    }

    public Chassis(SampleMecanumDrive drive, Gamepad g1) {
        this.drive = drive;
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        headingController.setInputBounds(-Math.PI, Math.PI);

        super.init("Chassis");
    }

    @Override
    public void init(HardwareMap map) {

    }

    @Override
    public void update(Gamepad g1, Gamepad g2, Telemetry telemetry) {
        Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();
        Pose2d driveDirection = new Pose2d();

        Double y = Math.copySign(Math.pow(-g1.left_stick_y, 3), -g1.left_stick_y);
        Double x = Math.copySign(Math.pow(-g1.left_stick_x, 3), -g1.left_stick_x);
        Double rotate = Math.copySign(Math.pow(-g1.right_stick_x, 3), -g1.right_stick_x);

        double target = 0;

        switch(mode) {
            case NORMAL:
                if(g1.a) {
                    mode = Mode.AIM;
                }

                if(g1.b) {
                    mode = Mode.AIM_NORMAL;
                }

                driveDirection = new Pose2d(y, x, rotate);
                break;
            case AIM_NORMAL:
                if(!g1.b) {
                    mode = Mode.NORMAL;
                }

                Vector2d fieldFrameInput = new Vector2d(
                        y, x
                );
                Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());
                Vector2d difference = targetPosition.minus(poseEstimate.vec());
                double theta = difference.angle();
                double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

                headingController.setTargetPosition(theta);
                double headingInput = (headingController.update(poseEstimate.getHeading()) * DriveConstants.kV + thetaFF) * DriveConstants.TRACK_WIDTH;
                driveDirection = new Pose2d(robotFrameInput, headingInput);
                break;
            case AIM:
                if(!g1.a) {
                    mode = Mode.NORMAL;
                }

                fieldFrameInput = new Vector2d(
                        y, x
                );
                robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());
                difference = targetPosition.minus(poseEstimate.vec());
                theta = difference.angle();
                target = Math.toDegrees(theta);

                if(target > 180)
                    target -= 360;

                if(target < -180)
                    target += 360;

                thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

                if(target < -30) {
                    target = -30;

                    headingController.setTargetPosition(Math.toRadians(target));
                    headingInput = (headingController.update(poseEstimate.getHeading()) * DriveConstants.kV + thetaFF) * DriveConstants.TRACK_WIDTH;
                    driveDirection = new Pose2d(robotFrameInput, headingInput);
                } else if(target > 30) {
                    target = 30;

                    headingController.setTargetPosition(Math.toRadians(target));
                    headingInput = (headingController.update(poseEstimate.getHeading()) * DriveConstants.kV + thetaFF) * DriveConstants.TRACK_WIDTH;
                    driveDirection = new Pose2d(robotFrameInput, headingInput);
                } else {
                    driveDirection = new Pose2d(robotFrameInput, rotate);
                }
                break;
        }

        drive.setWeightedDrivePower(driveDirection);
        headingController.update(poseEstimate.getHeading());

        distance = poseEstimate.vec().distTo(targetPosition);

        telemetry.addData("target", target);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
