package org.firstinspires.ftc.teamcode.utils.constants;

import static org.firstinspires.ftc.teamcode.utils.constants.AutoConstants.startX;
import static org.firstinspires.ftc.teamcode.utils.constants.AutoConstants.startY;

import com.acmerobotics.dashboard.config.Config;
@Config
public class BotConstants {
    public static double goalX = 122.68;
    public static double goalY = 123.84;
    public static double goalDY = 20; //correct later
    public static double CAMERA_YAW = 13;
    public enum BotState{
        MATH, MANUAL, TESTING
    }
}
