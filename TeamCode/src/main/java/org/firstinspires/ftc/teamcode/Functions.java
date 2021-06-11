package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class Functions {
    SampleMecanumDrive drive;
    Shooter shooter;

    public Functions(SampleMecanumDrive drive) {
        this.drive = drive;
    }

    public Functions(SampleMecanumDrive drive, Shooter shooter) {
        this.drive = drive;
        this.shooter = shooter;
    }

    public void sleepUpdate(Double ms) {
        ElapsedTime elapsedTime = new ElapsedTime();
        while (elapsedTime.milliseconds() < ms) {
            drive.update();
        }
    }

    public void sleepUpdate(int ms) {
        ElapsedTime elapsedTime = new ElapsedTime();
        while (elapsedTime.milliseconds() < ms) {
            drive.update();
        }
    }

    public void sleepUpdateShooter(Double ms) {
        ElapsedTime elapsedTime = new ElapsedTime();
        while (elapsedTime.milliseconds() < ms) {
            drive.update();
            shooter.update();
        }
    }

    public void sleepUpdateShooter(int ms) {
        ElapsedTime elapsedTime = new ElapsedTime();
        while (elapsedTime.milliseconds() < ms) {
            drive.update();
            shooter.update();
        }
    }
}
