package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class HardwareBase {
    public void init(String s) {
        System.out.print(s + " is initted.");
    }

    public abstract void init(HardwareMap map);
    public abstract void update(GamepadEx g1, GamepadEx g2, Telemetry telemetry);
}
