package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import
@Config
@TeleOp(name = "Limelight Tester")
public class LimelightTester extends LinearOpMode {

    // Dashboard-tunable
    public static int PIPELINE = 0;
    public static int DESIRED_TAG_ID = -1;         // -1 = best tag
    public static double DESIRED_FORWARD_M = 0.60; // for range error
    public static boolean USE_TAG_LOCK = false;

    @Override
    public void runOpMode() throws InterruptedException {
        MyTelem.init(telemetry);

        Robot robot = new Robot(hardwareMap, false);
        GamepadEx gp1 = new GamepadEx(gamepad1);

        gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            USE_TAG_LOCK = !USE_TAG_LOCK;
        });

        gp1.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            if (USE_TAG_LOCK) robot.limelightCamera.setDesiredTagId(DESIRED_TAG_ID);
            else robot.limelightCamera.clearDesiredTagId();
        });

        robot.follower.setStartingPose(new Pose(0, 0, 0));

        waitForStart();

        if (isStarted()) {
            robot.follower.startTeleopDrive();
        }

        while (opModeIsActive()) {
            robot.follower.setTeleOpMovementVectors(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true
            );


            if (USE_TAG_LOCK) robot.limelightCamera.setDesiredTagId(DESIRED_TAG_ID);
            else robot.limelightCamera.clearDesiredTagId();

            robot.update();

            boolean has = robot.limelightCamera.hasTarget();
            MyTelem.addData("LL Has Target", has);
            MyTelem.addData("LL Use Tag Lock", USE_TAG_LOCK);

            if (has) {
                MyTelem.addData("LL Tag ID", robot.limelightCamera.getTagId());
                MyTelem.addData("LL Distance (m)", robot.limelightCamera.getDistanceM());

                MyTelem.addData("LL Turn Error (deg)", robot.limelightCamera.getTurnErrorDeg());
                MyTelem.addData("LL Strafe Error (m)", robot.limelightCamera.getStrafeErrorM());
                MyTelem.addData("LL Range Error (m)", robot.limelightCamera.getRangeErrorM(DESIRED_FORWARD_M));

                LimelightCamera.TagTarget t = robot.limelightCamera.getTarget();
                MyTelem.addData("LL Pose X (m)", t.xM);
                MyTelem.addData("LL Pose Y (m)", t.yM);
                MyTelem.addData("LL Pose Z (m)", t.zM);
                MyTelem.addData("LL tx (deg)", t.txDeg);
                MyTelem.addData("LL ty (deg)", t.tyDeg);
                MyTelem.addData("LL area", t.area);
            }

            MyTelem.update();
        }

        robot.stop();
    }
}
