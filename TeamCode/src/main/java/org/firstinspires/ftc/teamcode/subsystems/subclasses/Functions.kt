package org.firstinspires.ftc.teamcode.subsystems.subclasses

import com.noahbres.jotai.StateMachine
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.Shooter

class Functions {
    lateinit var drive: SampleMecanumDrive
    lateinit var shooter: Shooter

    constructor(d: SampleMecanumDrive) {
        drive = d
    }

    constructor(d: SampleMecanumDrive, s: Shooter) {
        drive = d
        shooter = s
    }

    public fun sleepUpdate(ms: Double) {
        val elapsedTime = ElapsedTime()
        while (elapsedTime.milliseconds() < ms) {
            drive.update()
        }
    }

    public fun sleepUpdateShooter(ms: Double) {
        val elapsedTime = ElapsedTime()
        while(elapsedTime.milliseconds() < ms) {
            drive.update()
            shooter.update()
        }
    }
}