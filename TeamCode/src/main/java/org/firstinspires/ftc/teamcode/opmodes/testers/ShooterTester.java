package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.BlockerCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.KickerCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.ShooterCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.MyTelem;

@Config
@TeleOp(name = "Shooter Tester")

public class ShooterTester extends LinearOpMode {
    public void runOpMode(){
        MyTelem.init(telemetry);
        Robot robot = new Robot(hardwareMap, false);
        GamepadEx gp1 = new GamepadEx(gamepad1);
        robot.shooter.state = Shooter.ShooterState.TESTING;
        gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new ParallelCommandGroup(
                        new BlockerCommand(robot, Blocker.BlockerState.UNBLOCKED),
                        new IntakeCommand(robot, Intake.IntakeState.ON)
                )
        );
        

        gp1.getGamepadButton(GamepadKeys.Button.A).whenReleased(
                new ParallelCommandGroup(
                        new BlockerCommand(robot, Blocker.BlockerState.BLOCKED),
                        new IntakeCommand(robot, Intake.IntakeState.OFF)
                )
        );

        waitForStart();

        while (opModeIsActive()) {
            robot.update();
        }
        robot.stop();
    }
}
