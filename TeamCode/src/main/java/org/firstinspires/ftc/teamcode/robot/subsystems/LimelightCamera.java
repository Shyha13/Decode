package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.utils.MyTelem;

public class LimelightCamera implements Subsystem {
    public static class TagTarget {
        public boolean hasTarget = false;

        public int id = -1;
        public double txDeg = 0.0;
        public double tyDeg = 0.0;
        public double xM = 0.0;
        public double yM = 0.0;
        public double zM = 0.0;

        public double area = 0.0;

        public double distanceM() {
            return Math.sqrt(xM * xM + yM * yM + zM * zM);
        }
        public double forwardM() {
            return Math.abs(zM);
        }
        public double turnErrorDeg() {
            return txDeg;
        }
        public double strafeErrorM() {
            return xM;
        }
        public double rangeErrorM(double desiredForwardM) {
            return forwardM() - desiredForwardM;
        }
    }

    private final Limelight3A limelight;
    private final TagTarget target = new TagTarget();

    // Optional: lock onto a specific tag
    private int desiredTagId = -1;

    // Optional: keep LL polling rate reasonable
    private int pollRateHz = 50;

    public LimelightCamera(Limelight3A limelight, int pipelineIndex) {
        this.limelight = limelight;

        limelight.setPollRateHz(pollRateHz);
        limelight.pipelineSwitch(pipelineIndex);
        limelight.start();
    }

    public void setDesiredTagId(int id) {
        desiredTagId = id;
    }

    public void clearDesiredTagId() {
        desiredTagId = -1;
    }

    public void updateRobotYawDeg(double yawDeg) {
        limelight.updateRobotOrientation(yawDeg);
    }

    @Override
    public void periodic() {
        updateTarget();
        publishTelem();
    }

    private void updateTarget() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            target.hasTarget = false;
            target.id = -1;
            return;
        }

        java.util.List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            target.hasTarget = false;
            target.id = -1;
            return;
        }

        LLResultTypes.FiducialResult best = null;
        double bestScore = -1e9;

        for (LLResultTypes.FiducialResult fr : tags) {
            if (fr == null) continue;

            int id = fr.getFiducialId();
            if (desiredTagId >= 0 && id != desiredTagId) continue;

            double area = fr.getTargetArea();
            double score = area;

            if (score > bestScore) {
                bestScore = score;
                best = fr;
            }
        }

        if (best == null) {
            target.hasTarget = false;
            target.id = -1;
            return;
        }

        target.hasTarget = true;
        target.id = best.getFiducialId();
        target.area = best.getTargetArea();

        target.txDeg = best.getTargetXDegrees();
        target.tyDeg = best.getTargetYDegrees();

        Pose3D poseCam = best.getTargetPoseCameraSpace();
        if (poseCam != null) {
            Position p = poseCam.getPosition();
            target.xM = p.x;
            target.yM = p.y;
            target.zM = p.z;
        } else {
            target.xM = target.yM = target.zM = 0.0;
        }
    }

    public boolean hasTarget() {
        return target.hasTarget;
    }

    public int getTagId() {
        return target.id;
    }
    public double getDistanceM() {
        return target.distanceM();
    }

    public double getTurnErrorDeg() {
        return target.turnErrorDeg();
    }

    public double getStrafeErrorM() {
        return target.strafeErrorM();
    }

    public double getRangeErrorM(double desiredForwardM) {
        return target.rangeErrorM(desiredForwardM);
    }
    public TagTarget getTarget() {
        return target;
    }
    private void publishTelem() {
        MyTelem.addData("LL Has Target", target.hasTarget);
        if (!target.hasTarget) return;

        MyTelem.addData("LL Tag ID", target.id);
        MyTelem.addData("LL tx (deg)", target.txDeg);
        MyTelem.addData("LL ty (deg)", target.tyDeg);
        MyTelem.addData("LL x (m)", target.xM);
        MyTelem.addData("LL y (m)", target.yM);
        MyTelem.addData("LL z (m)", target.zM);
        MyTelem.addData("LL dist (m)", target.distanceM());
        MyTelem.addData("LL area", target.area);
    }
}
