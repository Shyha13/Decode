package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.KickerCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.utils.MyTelem;

@Config
@TeleOp(name = "Kicker Tester")
public class KickerTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MyTelem.init(telemetry);
        Robot robot = new Robot(hardwareMap, false);
        GamepadEx gp1 = new GamepadEx(gamepad1);

        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new KickerCommand(robot, Kicker.KickerState.ON));
        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(new KickerCommand(robot, Kicker.KickerState.OFF));
        robot.follower.setStartingPose(new Pose(0,0,0));

        waitForStart();

        if(isStarted()){
            robot.follower.startTeleopDrive();
        }

        while (opModeIsActive()) {
            robot.follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            MyTelem.addData("Kicker State", robot.kicker.state);
            robot.update();
        }
        robot.stop();
    }
}
