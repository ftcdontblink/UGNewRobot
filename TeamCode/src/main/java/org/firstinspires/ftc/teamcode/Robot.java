package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoal;

public class Robot {
    public SampleMecanumDrive drive;
    public Chassis mecanumDrive;
    public Intake intake;
    public Shooter shooter;
    public WobbleGoal wobbleGoal;

    public Robot(HardwareMap hardwareMap, GamepadEx g1, GamepadEx g2) {
        drive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive = new Chassis(drive, g1);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        wobbleGoal = new WobbleGoal(hardwareMap);
    }

    public void update(GamepadEx g1, GamepadEx g2) {
        drive.update();
        mecanumDrive.update(g1, g2);
        intake.update(g1, g2);
        shooter.update(g1, g2);
        wobbleGoal.update(g1, g2);
    }
}