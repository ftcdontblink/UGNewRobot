package org.firstinspires.ftc.teamcode.subsystems.subclasses;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;

public class VelocityPIDFController {
    private static final int ACCEL_SAMPLES = 3;
    private static final double VELOCITY_EPSILON = 20 + 1e-6;

    private PIDFController controller;

    private MovingStatistics accelSamples;

    private double lastPosition = Double.NaN;
    private double lastVelocity = Double.NaN;

    private double kV;
    private double kA;
    private double kStatic;

    DcMotorEx myMotor1;
    DcMotorEx myMotor2;

    public final ElapsedTime veloTimer = new ElapsedTime();
    private double lastTargetVelo = 0.0;

    private VelocityPIDFController() {}

    public VelocityPIDFController(PIDCoefficients pid) {
        this(pid, 0.0, 0.0, 0.0);
    }

    public VelocityPIDFController(PIDCoefficients pid, double kV) {
        this(pid, kV, 0.0, 0.0);
    }

    public VelocityPIDFController(PIDCoefficients pid, double kV, double kA) {
        this(pid, kV, kA, 0.0);
    }

    public VelocityPIDFController(PIDCoefficients pid, double kV, double kA, double kStatic) {
        controller = new PIDFController(pid);
        accelSamples = new MovingStatistics(ACCEL_SAMPLES);

        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;

        reset();
    }

    public VelocityPIDFController(PIDCoefficients pid, double kV, double kA, double kStatic, HardwareMap hardwareMap) {
        myMotor1 = hardwareMap.get(DcMotorEx.class, "flywheelMotor1");
        myMotor2 = hardwareMap.get(DcMotorEx.class, "flywheelMotor2");

        // Reverse as appropriate
        // myMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        // myMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Ensure that RUN_USING_ENCODER is not set
        myMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Turns on bulk reading
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        controller = new PIDFController(pid);
        accelSamples = new MovingStatistics(ACCEL_SAMPLES);

        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;

        reset();
    }

    public void setTargetAcceleration(double acceleration) {
        controller.setTargetVelocity(acceleration);
    }

    public void setTargetVelocity(double velocity) {
        controller.setTargetPosition(velocity);
    }

    private double calculateAccel(double measuredPosition, double measuredVelocity) {
        double dx = measuredPosition - lastPosition;
        if (dx != 0.0 && Math.abs(measuredVelocity - lastVelocity) > VELOCITY_EPSILON) {
            double accel = (measuredVelocity * measuredVelocity - lastVelocity * lastVelocity) / (2.0 * dx);

            lastPosition = measuredPosition;
            lastVelocity = measuredVelocity;

            accelSamples.add(accel);
        } else {
            accelSamples.add(0.0);
        }

        return accelSamples.getMean();
    }

    public double update(double measuredPosition, double measuredVelocity) {
        if (Double.isNaN(lastPosition)) {
            lastPosition = measuredPosition;
        }
        if (Double.isNaN(lastVelocity)) {
            lastVelocity = measuredVelocity;
        }

        double accel = calculateAccel(measuredPosition, measuredVelocity);

        double correction = controller.update(measuredVelocity, accel);
        double feedforward = Kinematics.calculateMotorFeedforward(
                controller.getTargetPosition(), controller.getTargetVelocity(), kV, kA, kStatic);
        return correction + feedforward;
    }

    public void setRPM(double rpm) {
        double targetVelo = rpm*28.0/60.0;

        // Call necessary controller methods
        this.setTargetVelocity(targetVelo);
        this.setTargetAcceleration((targetVelo - lastTargetVelo) / veloTimer.seconds());
        veloTimer.reset();

        lastTargetVelo = targetVelo;

        // Get the velocity from the motor with the encoder
        double motorPos = myMotor1.getCurrentPosition();
        double motorVelo = myMotor1.getVelocity();

        // Update the controller and set the power for each motor
        double power = this.update(motorPos, motorVelo);
        myMotor1.setPower(power);
        myMotor2.setPower(power);
    }

    public void reset() {
        controller.reset();

        lastPosition = Double.NaN;
        lastVelocity = Double.NaN;
    }
}
