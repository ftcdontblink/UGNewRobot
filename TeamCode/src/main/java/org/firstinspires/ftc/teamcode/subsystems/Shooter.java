package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.subclasses.SideHood;
import org.firstinspires.ftc.teamcode.subsystems.subclasses.TimedAction;
import org.firstinspires.ftc.teamcode.subsystems.subclasses.VelocityPIDFController;


@Config
public class Shooter {
    public static double theta = 0;

    public DcMotorEx flywheelLeft;
    public DcMotorEx flywheelRight;
    public DcMotorEx encoder;
    public Servo servo;

    public double distance = 70;

    public static double timing = 120;
    public static double velocity = 50;

    double targetVelo = 1620;

    private SHOOTER state = SHOOTER.SHOOTER_EMPTY;
    FtcDashboard dashboard;
    Telemetry dt;

    // Copy your feedforward gains here
    public static double kV = 0.0004275;
    public static double kA = 0.0002;
    public static double kStatic = 0;

    public static double rpm = 4200;
    static double power;

    public TimedAction servoBackAndForth;

    // Timer for calculating desired acceleration
    // Necessary for kA to have an affect

    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.00125, 0, 0);
    public final VelocityPIDFController veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);

    public final ElapsedTime veloTimer = new ElapsedTime();
    private double lastTargetVelo = 0.0;

    double motorVelo;

    public SideHood sd;

    public SampleMecanumDrive drive;
    public static double angle = 0.1;

    public enum SHOOTER {
        SHOOTER_FULL,
        SHOOTER_IDLE,
        SHOOTER_EMPTY,
        SHOOTER_POWERSHOTS,
        SHOOTER_AUTOMATE
    }

    public Shooter(HardwareMap hardwareMap) {
        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");
        servo = hardwareMap.get(Servo.class, "index");
        sd = new SideHood(hardwareMap);
        servo.setPosition(0.05);

        shooterConstants();
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
//coding is cool
        servoBackAndForth = new TimedAction(
                () -> servo.setPosition(0.2),
                () -> servo.setPosition(0.05),
                100, // ms
                true // runs symmetric
        );

        dashboard = FtcDashboard.getInstance();
        dt = dashboard.getTelemetry();

        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void shooterConstants() {
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void on(double velocity) {
        velocity = velocity * 28.0 / 60.0;
        flywheelRight.setVelocity(velocity);
        flywheelLeft.setVelocity(velocity);
    }

    public void off() {
        flywheelRight.setVelocity(0);
        flywheelLeft.setVelocity(0);
    }

    public void update(Gamepad gamepad) {
        sd.setPos(angle);
        switch (state) {
            case SHOOTER_EMPTY:
                rpm = 0;

                if (gamepad.y) {
                    state = SHOOTER.SHOOTER_FULL;
                }
                if (gamepad.dpad_down) {
                    state = SHOOTER.SHOOTER_POWERSHOTS;
                }

                break;
            case SHOOTER_FULL:
                rpm = 4000;
                if (gamepad.a) {
                    state = SHOOTER.SHOOTER_EMPTY;
                }
                if (gamepad.dpad_down) {
                    state = SHOOTER.SHOOTER_POWERSHOTS;
                }

                break;
            case SHOOTER_POWERSHOTS:
                rpm = 3400;

                if (gamepad.a) {
                    state = SHOOTER.SHOOTER_EMPTY;
                }
                if (gamepad.y) {
                    state = SHOOTER.SHOOTER_FULL;
                }

                break;
        }

        update();
        shoot(gamepad);
    }

    public void shoot(Gamepad gamepad) {
        if (gamepad.x && !servoBackAndForth.running()) servoBackAndForth.reset();
        servoBackAndForth.run();
    }

    public void shoot() {
        if (!servoBackAndForth.running()) servoBackAndForth.reset();
        servoBackAndForth.run();
    }

    public void update() {
        targetVelo = rpm * 28 / 1 / 60;

        // Call necessary controller methods
        veloController.setTargetVelocity(targetVelo);
        veloController.setTargetAcceleration((targetVelo - lastTargetVelo) / veloTimer.seconds());
        veloTimer.reset();

        lastTargetVelo = targetVelo;

        // Get the velocity from the motor with the encoder
        motorVelo = flywheelLeft.getVelocity();

        // Update the controller and set the power for each motor
        power = veloController.update(flywheelLeft.getCurrentPosition(), motorVelo);
        flywheelLeft.setPower(power);
        flywheelRight.setPower(power);
    }
}

