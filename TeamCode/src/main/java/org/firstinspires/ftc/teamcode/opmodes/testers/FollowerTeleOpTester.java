package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.MyTelem;

@TeleOp(name = "Follower TeleOp Tester")
public class FollowerTeleOpTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MyTelem.init(telemetry);

        Robot robot = new Robot(hardwareMap, false);
        robot.follower.setStartingPose(new Pose(0,0,0));
        GamepadEx gp1 = new GamepadEx(gamepad1);

        waitForStart();

        if(isStarted()){
            robot.follower.startTeleopDrive();
        }

        while (opModeIsActive()) {
            robot.follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            robot.update();
        }

        robot.stop();
    }
}
