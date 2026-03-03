package org.firstinspires.ftc.teamcode.utils.constants.auto;

import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.leverIntakeX;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.leverIntakeY;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.leverX;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.leverY;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.leverYNew;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.lvHead;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.lvX;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.lvY;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.nStart;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.s2ConX;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.s2ConY;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.s3ConX;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.s3ConY;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.s3Head;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.s3X;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.s3Y;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.startHeading;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.startX;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.startY;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;

@Config
public class CloseSideAutoPoseData {
    public static final Pose LEVER_INTAKE = new Pose(leverIntakeX, leverIntakeY);
    public static final Pose startPose = new Pose(startX, startY, Math.toRadians(nStart));
    public static final Pose shoot1 = new Pose(27.5,115.5,AutoConstants.shootingAngle);
    public static final Pose shootInt = new Pose(48,96,AutoConstants.shootingAngle);

    public static final Pose shoot = new Pose(AutoConstants.shootingX, AutoConstants.shootingY, AutoConstants.shootingAngle);

    public static final Pose s1 = new Pose (AutoConstants.s1X,AutoConstants.s1Y,AutoConstants.s1Head);
    public static final Pose s2 = new Pose (AutoConstants.s2X, AutoConstants.s2Y, AutoConstants.s2Head);

    public static final Pose s3 = new Pose (AutoConstants.s3X,AutoConstants.s3Y,AutoConstants.s3Head);

    public static final Pose s2Con = new Pose(s2ConX, s2ConY, Point.CARTESIAN);
    public static final Pose s3Con = new Pose(s3ConX, s3ConY, Point.CARTESIAN);

    public static final Pose lever = new Pose(lvX,lvY, Math.toRadians(lvHead));
    public static double mirrorX(double x, String color) {
        return color.equals("RED") ? 144 - x : x;
    }
    public static double mirrorHeading(double deg, String color) {
        return color.equals("RED") ? 180 - deg : deg;
    }
    public static Pose mirror(Pose p, String color) {
        return color.equals("RED") ? new Pose(144 - p.getX(), p.getY(), Math.toRadians(180 - Math.toDegrees(p.getHeading()))) : p;
    }
}
