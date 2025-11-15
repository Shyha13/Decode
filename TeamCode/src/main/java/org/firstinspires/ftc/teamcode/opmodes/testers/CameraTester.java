package org.firstinspires.ftc.teamcode.opmodes.testers;

import android.util.Size;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.CameraConstants;
import org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.utils.constants.TurretConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "AprilTag Detector", group = "Vision")
public class CameraTester extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private WebcamName camera;
    public static Boolean blue, red;
    public Servo turretLeftServo, turretRightServo;
    PIDController turretPID;


    private void setTurretPostition(double pos){
        turretLeftServo.setPosition(pos);
        turretRightServo.setPosition(pos);
    }
    private double getTurretPostition(){
        return turretLeftServo.getPosition();
    }



    @Override
    public void runOpMode() {
        blue = true;
        turretPID = new PIDController(CameraConstants.p, CameraConstants.i, CameraConstants.d);
        MyTelem.init(telemetry);
        initAprilTagProccessor(aprilTagProcessor, camera, CameraConstants.camaeraSizeX, CameraConstants.camaeraSizeY);
        turretLeftServo = hardwareMap.get(Servo.class, "turretLeftServo");
        turretRightServo = hardwareMap.get(Servo.class, "turretRightServo");
        camera = hardwareMap.get(WebcamName.class, "cameraOne");

        MyTelem.addData("Camera initialized", "");
        MyTelem.update();

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

            if (currentDetections.isEmpty()) {
                MyTelem.addData("No AprilTags detected.", "");
            }
            else {
                MyTelem.addData("Detected Tags:", currentDetections.size());
                for (AprilTagDetection detection : currentDetections) {
                    // TODO: ADD whether April tag is blue or red.
                    // TODO: ADD PID for Tracking using turret servo
                    // Todo: ADD rpm adjusting using Interpolation
                    // TODO: Make this a METHOD with parameters (bool blu or red, servo 1, servo 2)

                    double x = detection.ftcPose.x;
                    double y = detection.ftcPose.y;
                    double z = detection.ftcPose.z;
                    MyTelem.addData("ID: ", detection.id);
                    MyTelem.addData("Center: ","("+x+","+y+","+z+")");
                    MyTelem.addData("Pose X", x);
                    MyTelem.addData("Pose Y", y);
                    MyTelem.addData("Pose Z", z);
                    MyTelem.addData("Dist: ", detection.ftcPose.range);
                    MyTelem.addData("Elevation: ", detection.ftcPose.elevation);
                    if(blue && detection.id == CameraConstants.blueID){
                        MyTelem.addData("detecting blue", "");
                        turretPID.setPID(CameraConstants.p,CameraConstants.i,CameraConstants.d);
                        double correction = turretPID.calculate(x,0);
                        correction = Range.clip(getTurretPostition()+correction, 0, 1);
                        setTurretPostition(correction);
                    }
                }
            }

            MyTelem.update();
        }

        visionPortal.close();
    }

    public void initAprilTagProccessor(AprilTagProcessor processor, WebcamName camera, int x, int y) {
        processor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(camera)
                .addProcessor(processor)
                .setCameraResolution(new Size(x,y))
                .build();
    }

}
