package org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;

public class TurretCommand extends SequentialCommandGroup {
    public TurretCommand(Robot robot, Turret.TurretState state) {
//        if(state == Turret.TurretState.MATH_CAMERA){
//            if (Robot.getTargetTag().hasTarget) {
//                addCommands(
//                        new InstantCommand(() -> robot.turret.setState(Turret.TurretState.MATH_CAMERA))
//                );
//            } else{
//                addCommands(
//                        new InstantCommand(() -> robot.turret.setState(Turret.TurretState.MATH))
//                );
//            }
//            return;
//        }
        addCommands(
                new InstantCommand(() -> robot.turret.setState(state))
        );
    }
}
