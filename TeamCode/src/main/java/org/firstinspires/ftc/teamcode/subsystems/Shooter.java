package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.subclasses.Flicker;
import org.firstinspires.ftc.teamcode.subsystems.subclasses.RingCounter;
import org.firstinspires.ftc.teamcode.subsystems.subclasses.SideHood;
import org.firstinspires.ftc.teamcode.subsystems.subclasses.VelocityPIDFController;

@Config
public class Shooter extends HardwareBase {
    public VelocityPIDFController controller;
    public static double rpm = 0;

    public InterpLUT veloAdjust;
    public InterpLUT veloAdjustPowershots;

    public SideHood sideHood;
    public Flicker flicker;

    State state = State.ON;
    public RingCounter counter;

    public enum State {
        OFF,
        IDLE,
        ON,
        POWERSHOTS,
    }

    public Shooter(HardwareMap hardwareMap) {
        init(hardwareMap);
        super.init("Shooter");
    }

    @Override
    public void init(HardwareMap map) {
//        sideHood = new SideHood(map);
        flicker = new Flicker(map);
//        counter = new RingCounter(map, false);

        controller = new VelocityPIDFController(new PIDCoefficients(0.00125, 0, 0), 0.0004275, 0.00015, 0, map);

//        veloAdjust = new InterpLUT();
//        veloAdjustPowershots = new InterpLUT();
//
//
//        veloAdjust.createLUT();
//        veloAdjustPowershots.createLUT();
    }

    @Override
    public void update(GamepadEx g1, GamepadEx g2, Telemetry telemetry) {
        controller.setRPM(rpm);
//        sideHood.update(2, g1, g2);

        flicker.kick(g1);
    }
}
