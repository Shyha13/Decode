package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.ShooterCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.MyTelem;

@Config
@TeleOp(name = "Shooter Tester")
public class ShooterTester extends LinearOpMode {
    public void runOpMode(){
        MyTelem.init(telemetry);
        Robot robot = new Robot(hardwareMap, false);
        GamepadEx gp1 = new GamepadEx(gamepad1);

        gp1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new ShooterCommand(robot, Shooter.ShooterState.FAR));
        gp1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(new ShooterCommand(robot, Shooter.ShooterState.STOP));
        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new ShooterCommand(robot, Shooter.ShooterState.CLOSE));
        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(new ShooterCommand(robot, Shooter.ShooterState.STOP));

        waitForStart();

        while (opModeIsActive()) {
            robot.update();
        }
        robot.stop();
    }
}
