//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package org.firstinspires.ftc.teamcode.pedroPathing.customLocalizer;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.constants.PinpointConstants;
import com.pedropathing.localization.localizers.PinpointLocalizer;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Vector;
import com.pedropathing.util.NanoTimer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Objects;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utils.PoseParameters;

public class CustomLocalizerPinpointV2 extends Localizer {
    private HardwareMap hardwareMap;
    private GoBildaPinpointDriver odo;
    private double previousHeading;
    private double totalHeading;
    private Pose startPose;
    private long deltaTimeNano;
    private NanoTimer timer;
    private Pose currentVelocity;
    private Pose pinpointPose;

    public CustomLocalizerPinpointV2(HardwareMap map) {
        this(map, new Pose());
    }

    public CustomLocalizerPinpointV2(HardwareMap map, Pose setStartPose) {
        this.odo = (GoBildaPinpointDriver)this.hardwareMap.get(GoBildaPinpointDriver.class, PinpointConstants.hardwareMapName);
        this.setOffsets(PinpointConstants.forwardY, PinpointConstants.strafeX, PinpointConstants.distanceUnit);
        if (PinpointConstants.useYawScalar) {
            this.odo.setYawScalar(PinpointConstants.yawScalar);
        }

        this.odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        this.odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        this.setStartPose(setStartPose);
        this.totalHeading = (double)0.0F;

        this.pinpointPose = this.startPose;
        this.currentVelocity = new Pose();
        this.previousHeading = setStartPose.getHeading();
    }

    public Pose getPose() {
        return this.pinpointPose.copy();
    }

    public Pose getVelocity() {
        return this.currentVelocity.copy();
    }

    public Vector getVelocityVector() {
        return this.currentVelocity.getVector();
    }

    public void setStartPose(Pose setStart) {
        if (!Objects.equals(this.startPose, new Pose()) && this.startPose != null) {
            Pose currentPose = MathFunctions.subtractPoses(MathFunctions.rotatePose(this.pinpointPose, -this.startPose.getHeading(), false), this.startPose);
            this.setPose(MathFunctions.addPoses(setStart, MathFunctions.rotatePose(currentPose, setStart.getHeading(), false)));
        } else {
            this.setPose(setStart);
        }

        this.startPose = setStart;
    }

    public void setPose(Pose setPose) {
        this.odo.setPosition(new Pose2D(DistanceUnit.INCH, setPose.getX(), setPose.getY(), AngleUnit.RADIANS, setPose.getHeading()));
        this.pinpointPose = setPose;
        this.previousHeading = setPose.getHeading();
    }

    public void update() {
        this.odo.update();
        Pose currentPinpointPose = PoseParameters.pose2DToPose(this.odo.getPosition());
        this.totalHeading += MathFunctions.getSmallestAngleDifference(currentPinpointPose.getHeading(), this.previousHeading) * MathFunctions.getTurnDirection(previousHeading, currentPinpointPose.getHeading());
        this.previousHeading = currentPinpointPose.getHeading();
        currentVelocity = new Pose(odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH), odo.getHeadingVelocity(AngleUnit.RADIANS.getUnnormalized()));
        this.pinpointPose = currentPinpointPose;
    }

    public double getTotalHeading() {
        return this.totalHeading;
    }

    public double getForwardMultiplier() {
        return (double)this.odo.getEncoderY();
    }

    public double getLateralMultiplier() {
        return (double)this.odo.getEncoderX();
    }

    public double getTurningMultiplier() {
        return (double)this.odo.getYawScalar();
    }

    private void setOffsets(double xOffset, double yOffset, DistanceUnit unit) {
        this.odo.setOffsets(xOffset, yOffset, unit);
    }

    public void resetIMU() {
        resetPinpoint();
    }

    private void resetPinpoint() {
        this.odo.resetPosAndIMU();

        try {
            Thread.sleep(300L);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public boolean isNAN() {
        return Double.isNaN(getPose().getX()) || Double.isNaN(getPose().getY()) || Double.isNaN(getPose().getHeading());
    }
    public GoBildaPinpointDriver getPinpoint(){
        return odo;
    }
}
