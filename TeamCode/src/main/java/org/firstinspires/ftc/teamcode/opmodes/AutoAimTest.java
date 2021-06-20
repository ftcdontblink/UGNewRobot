package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

/**
 * This opmode demonstrates how one would implement "align to point behavior" in teleop. You specify
 * a desired vector (x/y coordinate) via`targetPosition`. In the `ALIGN_TO_POINT` mode, the bot will
 * switch into field centric control and independently control its heading to align itself with the
 * specified `targetPosition`.
 * <p>
 * Press `a` to switch into alignment mode and `b` to switch back into standard teleop driving mode.
 * <p>
 * Note: We don't call drive.update() here because it has its own field drawing functions. We don't
 * want that to interfere with our graph so we just directly update localizer instead
 */
@Config
@TeleOp(group = "advanced")
public class AutoAimTest extends LinearOpMode {

    public static double DRAWING_TARGET_RADIUS = 2;
    public static double b = 0.18;
    public static Vector2d shooterOffset = new Vector2d(5, -6);

    // Define 2 states, driver control or alignment control
    enum Mode {
        NORMAL_CONTROL,
        ALIGN_TO_POINT
    }

    enum Colors {
        RED,
        BLUE
    }

    Colors colors = Colors.RED;

    private Mode currentMode = Mode.NORMAL_CONTROL;

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    // Declare a target vector you'd like your bot to align with
    // Can be any x/y coordinate of your choosing
    private Vector2d targetPositionRed = new Vector2d(72, -36);
    private Vector2d targetPositionBlue = new Vector2d(72, 36);

    Servo sidehood;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        sidehood = hardwareMap.get(Servo.class, "sidehood");
        sidehood.setPosition(0.09);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.getLocalizer().setPoseEstimate(new Pose2d(-63.5, -14, 0));

        // Set input bounds for the heading controller
        // Automatically handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);

        double offset = 0;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            if(gamepad1.right_trigger > 0) {

            }

            double angleDiff = 0;
            // Read pose
            Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();

            // Declare a drive direction
            // Pose representing desired x, y, and angular velocity
            Pose2d driveDirection = new Pose2d();

            telemetry.addData("mode", currentMode);

            // Declare telemetry packet for dashboard field drawing
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            robot.update(gamepad1, gamepad2, true);

            if(gamepad2.left_bumper) {
                colors = Colors.BLUE;
            }

            if(gamepad2.right_bumper) {
                colors = Colors.RED;
            }

            switch (currentMode) {
                case NORMAL_CONTROL:
                    // Switch into alignment mode if `a` is pressed
                    if (gamepad1.a) {
                        currentMode = Mode.ALIGN_TO_POINT;
                    }

                    angleDiff = 0;

                    // Standard teleop control
                    // Convert gamepad input into desired pose velocity
                    driveDirection = new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    );

                    sidehood.setPosition(b);
                    break;
                case ALIGN_TO_POINT:
                    // Switch back into normal driver control mode if `b` is pressed
                    if (!gamepad1.a) {
                        currentMode = Mode.NORMAL_CONTROL;
                    }

                    // Create a vector from the gamepad x/y inputs which is the field relative movement
                    // Then, rotate that vector by the inverse of that heading for field centric control
                    Vector2d fieldFrameInput = new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    );

                    // Difference between the target vector and the bot's position



                    if(colors == Colors.BLUE) {
                        Vector2d difference = targetPositionBlue.minus(poseEstimate.vec().plus(shooterOffset.rotated(poseEstimate.getHeading())));
                        Shooter.distance = difference.norm();
                        // Obtain the target angle for feedback and derivative for feedforward
                        double theta = difference.angle();

                        angleDiff = Angle.normDelta(theta - poseEstimate.getHeading());
                        telemetry.addData("Angle diff", angleDiff);
                    } else {
                        Vector2d difference = targetPositionRed.minus(poseEstimate.vec().plus(shooterOffset.rotated(poseEstimate.getHeading())));
                        Shooter.distance = difference.norm();
                        // Obtain the target angle for feedback and derivative for feedforward
                        double theta = difference.angle();

                        angleDiff = Angle.normDelta(theta - poseEstimate.getHeading());
                        telemetry.addData("Angle diff", angleDiff);
                    }

                    double a = -1.0/300.0;
                    double x = Math.toDegrees(angleDiff);
                    double position = a*x+b;

                    if(position < 0.08 || position > 0.25) {
                        sidehood.setPosition(position);
                    } else {
                        sidehood.setPosition(b);
                    }

                    driveDirection = new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    );


                    telemetry.update();
                    // Not technically omega because its power. This is the derivative of atan2

                    // Set the target heading for the heading controller to our desired angle
//                    headingController.setTargetPosition(theta);
//
//                    // Set desired angular velocity to the heading controller output + angular
//                    // velocity feedforward
//                    double headingInput = (headingController.update(poseEstimate.getHeading())
//                            * DriveConstants.kV + thetaFF)
//                            * DriveConstants.TRACK_WIDTH;
//
//                    // Combine the field centric x/y velocity with our derived angular velocity
//                    driveDirection = new Pose2d(
//                            robotFrameInput,
//                            headingInput
//                    );
//
//                    // Draw the target on the field
//                    fieldOverlay.setStroke("#dd2c00");
//                    fieldOverlay.strokeCircle(targetPosition.getX(), targetPosition.getY(), DRAWING_TARGET_RADIUS);
//
//                    // Draw lines to target
//                    fieldOverlay.setStroke("#b89eff");
//                    fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), poseEstimate.getX(), poseEstimate.getY());
//                    fieldOverlay.setStroke("#ffce7a");
//                    fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), targetPosition.getX(), poseEstimate.getY());
//                    fieldOverlay.strokeLine(targetPosition.getX(), poseEstimate.getY(), poseEstimate.getX(), poseEstimate.getY());
                    break;
            }

//            // Draw bot on canvas
//            fieldOverlay.setStroke("#3F51B5");
//            DashboardUtil.drawRobot(fieldOverlay, poseEstimate);

            drive.setWeightedDrivePower(driveDirection);

            // Update the heading controller with our current heading
//            headingController.update(poseEstimate.getHeading());

            // Update he localizer
            drive.getLocalizer().update();

            telemetry.addData("angle diff", angleDiff);

//            // Send telemetry packet off to dashboard
//            FtcDashboard.getInstance().sendTelemetryPacket(packet);
//
//            // Print pose to telemetry
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());
//            telemetry.update();
        }
    }
}