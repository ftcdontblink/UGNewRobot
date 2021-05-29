package org.firstinspires.ftc.teamcode.subsystems.subclasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Flicker {
    TimedAction flicker;
    Servo servo;

    public static double startPos = 0;
    public static double endPos = 0.15;

    public Flicker(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "flicker");
        flicker = new TimedAction(
                () -> servo.setPosition(endPos),
                () -> servo.setPosition(startPos),
                5,
                5
        );
    }

    public void flick() {
        if (!flicker.running()) {
            flicker.reset();
        }
        flicker.run();
    }

    public void flick(double num) {
        int counter = 0;
        while(counter < num) {
            flick();
        }
    }
}