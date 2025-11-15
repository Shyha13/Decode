package org.firstinspires.ftc.teamcode.robot.subsystems;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.utils.constants.TurretConstants;

public class Turret implements Subsystem {
    public Servo turretLeftServo, turretRightServo;
    public TurretState state;
    public Turret(Servo turretLeftServo, Servo turretRightServo){
        this.turretLeftServo = turretLeftServo;
        this.turretRightServo = turretRightServo;

        state = TurretState.FRONT;
    }
    public void setState(TurretState state){
        this.state = state;
        switch (state){
            case FRONT:
                turretLeftServo.setPosition(TurretConstants.turretForwardPosition);
                turretRightServo.setPosition(TurretConstants.turretForwardPosition);
                break;
            case BACK:
                turretLeftServo.setPosition(TurretConstants.turretBackPosition);
                turretRightServo.setPosition(TurretConstants.turretBackPosition);
            case MATH:
                break;
        }
    }

    public TurretState getState(){
        return state;
    }
    public enum TurretState{
        FRONT, MATH, BACK
    }

}
