package org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands;

import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;

public class ShooterCommand extends SequentialCommandGroup {
    public ShooterCommand(Robot robot, Shooter.ShooterState state){
        addCommands(
                new InstantCommand(() -> robot.shooter.setState(state))
        );
    }
}
