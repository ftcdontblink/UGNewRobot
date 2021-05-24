package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.subclasses.VelocityPIDFController;

public class Shooter extends HardwareBase {
    public VelocityPIDFController controller;

    public Shooter(HardwareMap hardwareMap) {
        super.init("Shooter");
    }

    @Override
    public void init(HardwareMap map) {
        controller = new VelocityPIDFController(new PIDCoefficients(0, 0, 0), 0, 0, 0, map);
    }

    @Override
    public void update(GamepadEx g1, GamepadEx g2) {
        controller.setRPM(4000);
    }
}
