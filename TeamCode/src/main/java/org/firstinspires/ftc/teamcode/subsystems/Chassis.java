package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Chassis extends HardwareBase {
    public static Vector2d targetPosition = new Vector2d(72, -36);
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);
    SampleMecanumDrive drive;
    Mode mode = Mode.NORMAL;
    ToggleButtonReader toggleA;

    enum Mode {
        NORMAL,
        AIM
    }

    public Chassis(SampleMecanumDrive drive, GamepadEx g1) {
        this.drive = drive;
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        headingController.setInputBounds(-Math.PI, Math.PI);

        toggleA = new ToggleButtonReader(g1, GamepadKeys.Button.A);

        super.init("Chassis");
    }

    @Override
    public void init(HardwareMap map) {

    }

    @Override
    public void update(GamepadEx g1, GamepadEx g2) {
        Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();
        Pose2d driveDirection = new Pose2d();

        Double y = Math.copySign(Math.pow(-g1.getLeftY(), 2), -g1.getLeftY());
        Double x = Math.copySign(Math.pow(-g1.getLeftX(), 2), -g1.getLeftX());
        Double rotate = Math.copySign(Math.pow(-g1.getRightX(), 2), -g1.getRightX());

        switch(mode) {
            case NORMAL:
                if(toggleA.getState()) {
                    mode = Mode.AIM;
                }

                driveDirection = new Pose2d(y, x, rotate);
                break;
            case AIM:
                if(!toggleA.getState()) {
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
    }
}
