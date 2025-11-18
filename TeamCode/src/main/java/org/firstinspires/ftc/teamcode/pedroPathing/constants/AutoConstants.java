package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class AutoConstants {
    public static double shootingAngle = 135;
    public static int firstWait = 300;
    public static int secondWait = 700;
    public static int thirdWait = 1800;
    public static double shootingX = 57;
    public static double shootingY = 82;
    public static double firstIntakeX = 19;
    public static double secondIntakeX = 10.000;
    public static Pose finalPose = new Pose(0,0,0);
    public static int intakeBallWait = 300;
}
