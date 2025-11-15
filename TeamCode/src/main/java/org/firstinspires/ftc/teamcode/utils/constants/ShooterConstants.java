package org.firstinspires.ftc.teamcode.utils.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    public static double closeShootRPM = 1300;// 1800
    public static double closeShootCounterRollerRPM = 2000;// TESstING NECESSARY
    public static double closeShootAutoRPM = 1750;
    public static double farShootRPM = 2200; //2.16k
    public static double farShootCounterRollerRPM = 2750;//TESTING NEcESSARY
    public static double kf = 0.3, kp = 1.6, ki = 0, kd = 0.00004;
    public static double tuningTestingRPM = 0;
    public static int TICKS_PER_REV = 28;
    public static int MAX_RPM = 6000;
    public static double CRkf = 0.3, CRkp = 1.6, CRki = 0, CRkd = 0.00004;
    public static int CR_TICKS_PER_REV = 28;



}
