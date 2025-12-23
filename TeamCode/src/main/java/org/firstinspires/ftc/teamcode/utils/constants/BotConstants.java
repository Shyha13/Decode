package org.firstinspires.ftc.teamcode.utils.constants;

import com.acmerobotics.dashboard.config.Config;
@Config
public class BotConstants {
    public static double goalX = 144;
    public static double goalY = 144;
    public static double goalDY = 20; //correct later
    public static double CAMERA_YAW = 13;
    public enum BotState{
        MATH, MANUAL, TESTING
    }
}
