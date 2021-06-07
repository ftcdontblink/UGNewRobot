package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
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

    public static double distance = 0;

    enum Mode {
        NORMAL,
        AIM
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

        Double y = Math.copySign(Math.pow(-g1.left_stick_y, 2), -g1.left_stick_y);
        Double x = Math.copySign(Math.pow(-g1.left_stick_x, 2), -g1.left_stick_x);
        Double rotate = Math.copySign(Math.pow(-g1.right_stick_x, 2), -g1.right_stick_x);

        switch(mode) {
            case NORMAL:
                if(g1.a) {
                    mode = Mode.AIM;
                }

                driveDirection = new Pose2d(y, x, rotate);
                break;
            case AIM:
                if(!g1.a) {
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
        }

        drive.setWeightedDrivePower(driveDirection);
        headingController.update(poseEstimate.getHeading());

        distance = poseEstimate.vec().distTo(targetPosition);
    }
}
