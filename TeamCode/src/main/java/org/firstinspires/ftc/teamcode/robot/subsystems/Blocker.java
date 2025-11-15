package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.constants.BlockerConstants;
import org.firstinspires.ftc.teamcode.utils.constants.IntakeConstants;

public class Blocker implements Subsystem {
    public BlockerState state;
    public Servo blockerServo;

    public Blocker(Servo blockerServo) {
        this.blockerServo = blockerServo;
        state = BlockerState.BLOCKED;
    }

    public void setState(BlockerState state){
        this.state = state;
        switch (state){
            case BLOCKED:
                blockerServo.setPosition(BlockerConstants.Blocked);
                break;
            case UNBLOCKED:
                blockerServo.setPosition(BlockerConstants.Unblocked);
                break;
            default:
                blockerServo.setPosition(BlockerConstants.Blocked);
                break;
        }
    }

    public BlockerState getState(){
        return state;
    }
    public enum BlockerState{
        BLOCKED, UNBLOCKED;
    }

}
