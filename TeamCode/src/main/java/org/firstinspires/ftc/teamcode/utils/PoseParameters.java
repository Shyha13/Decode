package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PoseParameters {
    public static Pose mirror(Pose pose){
        return new Pose(144 - pose.getX(), 144 - pose.getY(), Math.PI - pose.getHeading());
    }

    public static Pose2D poseToPose2D(Pose pose) {
        return new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading());
    }

    public static Pose pose2DToPose(Pose2D pose) {
        return new Pose(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.RADIANS));
    }
}
