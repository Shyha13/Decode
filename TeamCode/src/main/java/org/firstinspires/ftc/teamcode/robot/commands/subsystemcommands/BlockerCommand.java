package org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands;

import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;

public class BlockerCommand extends SequentialCommandGroup {
    public BlockerCommand(Robot robot, Blocker.BlockerState state){
        addCommands(
                new InstantCommand(() -> robot.blocker.setState(state)),
                new WaitCommand(100)
        );
    }
}
