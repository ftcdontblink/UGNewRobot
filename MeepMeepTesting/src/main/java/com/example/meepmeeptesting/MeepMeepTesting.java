package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

import java.util.Arrays;

public class MeepMeepTesting {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels

        MeepMeep mm = new MeepMeep(1000)
                // Set field image
                .setBackground(MeepMeep.Background.FIELD_ULTIMATE_GOAL_DARK)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-63.5, -14.5, 0))
                                .splineToConstantHeading(new Vector2d(-5, -14), 0)
                                .splineTo(new Vector2d(36-20, -8), 0)
                                .splineTo(new Vector2d(63.5-24, -58), 3*Math.PI/2)
                                .turn(Math.PI/2)
                                .forward(20)
                                .turn(Math.PI/2)
                                .forward(50)
                                .splineToLinearHeading(new Pose2d(-10, -36 ,0), Math.PI)

                                .lineTo(new Vector2d(-32, -36), new MinVelocityConstraint(
                                                Arrays.asList(
                                                        new AngularVelocityConstraint(Math.toRadians(180)),
                                                        new MecanumVelocityConstraint(7.5, 15)
                                                )),
                                        new ProfileAccelerationConstraint(52.48291908330528)
                                )
                                .waitSeconds(1)
                                .lineTo(new Vector2d(53.5, -46))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(-5, -36))
                                .build()
                )
                .start();
    }
}