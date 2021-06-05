package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

public class MeepMeepTesting {
    public static double MAX_VEL = 62.979502899966334;
    public static double MAX_ACCEL = 62.979502899966334;
    public static double MAX_ANG_VEL = Math.toRadians(207.3827420689655);
    public static double MAX_ANG_ACCEL = Math.toRadians(207.3827420689655);

    public static Pose2d startPose = new Pose2d(-63.5, -14.5, 0);

    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels

        MeepMeep mm = new MeepMeep(800)
                // Set field image
                .setBackground(MeepMeep.Background.FIELD_ULTIMATE_GOAL_DARK)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .splineToConstantHeading(new Vector2d(-5, -14), 0)
                                .splineToConstantHeading(new Vector2d(36, -19), 0)
                                .lineTo(new Vector2d(36, -62))
                                .lineTo(new Vector2d(36, -14))
                                .lineTo(new Vector2d(5, -14))
                                .build()
                )
                .start();
    }
}