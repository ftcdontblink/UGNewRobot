package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    public FtcDashboard dashboard;
    public Telemetry telemetry;

    public Robot(HardwareMap hardwareMap, Gamepad g1, Gamepad g2) {
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        drive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive = new Chassis(drive, g1);
        mecanumDrive.init(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        wobbleGoal = new WobbleGoal(hardwareMap);
    }

    public void update(Gamepad g1, Gamepad g2) {
        mecanumDrive.update(g1, g2, telemetry);
        intake.update(g1, g2, telemetry);
        shooter.update(g2);
        wobbleGoal.update(g1, g2, telemetry);
    }
}
