package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.localization.Pose;

public class PoseParameters {
    public static Pose mirror(Pose pose){
        return new Pose(144 - pose.getX(), 144 - pose.getY(), Math.PI - pose.getHeading());
    }
}
