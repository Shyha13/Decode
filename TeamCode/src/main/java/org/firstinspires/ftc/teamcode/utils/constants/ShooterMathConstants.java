package org.firstinspires.ftc.teamcode.utils.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterMathConstants {
    public static double SCORE_HEIGHT = 49.2 - 11.07;
    public static double SCORE_HEIGHT_AUTO = 28;
    public static double SCORE_AUTO_ANGLE = -0.15;

    public static double SCORE_ANGLE = -0.2;
    public static double PASS_THROUGH_POINT_RADIUS = 5;
    public static double HOOD_MAX_ANGLE = Math.toRadians(70); //0.71 servo pos
    public static double HOOD_MIN_ANGLE = Math.toRadians(35); //0.17 servo pos
    public static double perpMultiplier = 0;
    public static double turretMultiplier = 0;
}
