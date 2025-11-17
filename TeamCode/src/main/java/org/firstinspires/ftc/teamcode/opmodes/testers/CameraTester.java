package org.firstinspires.ftc.teamcode.opmodes.testers;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@TeleOp(name = "AprilTag Turret Tracker", group = "Vision")
public class CameraTester extends LinearOpMode {

    private VisionPortal portal;
    private AprilTagProcessor tags;

    Servo left, right;

    // Turret mapping
    double OFFSET = 0.5;
    double SLOPE = 0.0032222;
    double smooth = 0.85;

    // Scanning mode (when tag lost)
    double scanSpeed = 0.002;       // servo speed per loop
    boolean scanRight = true;

    @Override
    public void runOpMode() {

        left  = hardwareMap.get(Servo.class, "turretLeftServo");
        right = hardwareMap.get(Servo.class, "turretRightServo");

        WebcamName cam = hardwareMap.get(WebcamName.class, "camOne");

        tags = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(cam)
                .addProcessor(tags)
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .build();

        telemetry.addLine("READY — Point at tag");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            List<AprilTagDetection> dets = tags.getDetections();

            if (!dets.isEmpty()) {
                AprilTagDetection det = dets.get(0);  // track first tag

                double x = det.ftcPose.x;
                double z = det.ftcPose.z;

                // horizontal bearing: left/right
                double yawDeg = Math.toDegrees(Math.atan2(x, z));

                // convert to servo pos
                double target = OFFSET + SLOPE * yawDeg;
                target = Range.clip(target, 0, 1);

                double old = left.getPosition();
                double newPos = smooth * old + (1 - smooth) * target;

                left.setPosition(newPos);
                right.setPosition(newPos);

                // reset scan when tag seen
                scanRight = (yawDeg > 0);

                telemetry.addData("Mode", "Tracking tag");
                telemetry.addData("Yaw (deg)", yawDeg);
                telemetry.addData("ServoPos", newPos);
            }
            else {
                // ========= SCANNING (tag lost) =========
                double pos = left.getPosition();

                if (scanRight) pos += scanSpeed;
                else           pos -= scanSpeed;

                // bounce at edges
                if (pos > 1.0) { pos = 1.0; scanRight = false; }
                if (pos < 0.0) { pos = 0.0; scanRight = true; }

                left.setPosition(pos);
                right.setPosition(pos);

                telemetry.addData("Mode", "Scanning to find tag");
                telemetry.addData("ServoPos", pos);
            }

            telemetry.update();
            sleep(20);
        }

        portal.close();
    }
}
