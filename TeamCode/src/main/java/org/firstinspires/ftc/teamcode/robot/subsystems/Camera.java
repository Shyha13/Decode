package org.firstinspires.ftc.teamcode.robot.subsystems;


import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.CameraConstants;
import org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
public class Camera implements Subsystem {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private WebcamName camera;
    public static Boolean blue, red;
    public Servo turretLeftServo, turretRightServo;
    PIDController turretPID;

    public CameraState state = CameraState.DETECTOFF;
    public Camera(WebcamName camera, int x, int y) {
        turretPID = new PIDController(CameraConstants.p, CameraConstants.i, CameraConstants.d);
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(camera)
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(x,y))
                .build();
    }
    public VisionPortal getVisionPortal(){
        return visionPortal;
    }
    public void setState(CameraState state){
        switch (state) {
            case DETECTOFF:
                visionPortal.stopStreaming();
                break;
            case DETECTBLUE:

                if(visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING){
                    visionPortal.resumeStreaming();
                }

                List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

                for(AprilTagDetection detection : currentDetections){
                    if(detection.id == CameraConstants.blueID){

                        double x = detection.ftcPose.x;

                        MyTelem.addData("detecting blue", "");

                        turretPID.setPID(CameraConstants.p,CameraConstants.i,CameraConstants.d);
                        double correction = turretPID.calculate(x,0);
                        correction = Range.clip(getTurretPostition()+correction, 0, 1);
                        setTurretPostition(correction);
                    }
                    else{MyTelem.addData("blue aprilTag not detected", "");}
                }
                break;
            case DETECTRED:

                if(visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING){
                    visionPortal.resumeStreaming();
                }

                List<AprilTagDetection> currentDetectionsRed = aprilTagProcessor.getDetections();

                for(AprilTagDetection detection : currentDetectionsRed){
                    if(detection.id == CameraConstants.redID){

                        double x = detection.ftcPose.x;

                        MyTelem.addData("detecting red", "");

                        turretPID.setPID(CameraConstants.p,CameraConstants.i,CameraConstants.d);
                        double correction = turretPID.calculate(x,0);
                        correction = Range.clip(getTurretPostition()+correction, 0, 1);
                        setTurretPostition(correction);
                    }
                    else{MyTelem.addData("red aprilTag not detected", "");}
                }
                break;
            case DETECTPATTERN:
                break;

        }
    }
    private void setTurretPostition(double pos){
        turretLeftServo.setPosition(pos);
        turretRightServo.setPosition(pos);
    }
    private double getTurretPostition(){
        return turretLeftServo.getPosition();
    }

    @Override
    public void periodic(){
        setState(state);
        turretPID.setPID(CameraConstants.p, CameraConstants.i, CameraConstants.d);
    }


    public enum CameraState {
        DETECTRED, DETECTBLUE, DETECTPATTERN, DETECTOFF
    }
}
