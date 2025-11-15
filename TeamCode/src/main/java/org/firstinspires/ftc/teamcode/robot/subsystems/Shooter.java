package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants;

@Config
public class Shooter implements Subsystem {
    DcMotorEx topShooter, bottomShooter, counterRoller;
    Servo hood;
    PIDController shooterrpmPID, counterRollerrpmPID;

    public ShooterState state = ShooterState.STOP;
    public Shooter(DcMotorEx topShooter, DcMotorEx bottomShooter, DcMotorEx counterRoller, Servo hood){
        shooterrpmPID = new PIDController(ShooterConstants.kp, ShooterConstants.ki, ShooterConstants.kd);
        shooterrpmPID.setTolerance(10);
        counterRollerrpmPID = new PIDController(ShooterConstants.CRkp, ShooterConstants.CRki, ShooterConstants.CRkd);
        counterRollerrpmPID.setTolerance(10);

        this.topShooter = topShooter;
        this.bottomShooter = bottomShooter;
        this.counterRoller = counterRoller;
        this.hood = hood;
    }

    public void setState(ShooterState state){
        this.state = state;
//        switch (state) {
//            case SHOOTING:
//                ShooterInput constants = ShooterCalculations.getHoodAndPower();
//                setPIDPower(constants.getMotorRPM());
//                setHood(constants.getHoodAngle());
//            case STOP:
//                setPIDPower(0);
//        }
        switch (state) {
            case CLOSE:
                setShooterPIDPower(ShooterConstants.closeShootRPM);
                setCounterRollerPIDPower(ShooterConstants.closeShootRPM);
                break;
            case FAR:
                setShooterPIDPower(ShooterConstants.farShootRPM);
                setCounterRollerPIDPower(ShooterConstants.farShootCounterRollerRPM);
                break;
            case STOP:
                setShooterPIDPower(0);
                setCounterRollerPIDPower(0);
                break;
            case TESTING:
                setShooterPIDPower(ShooterConstants.tuningTestingRPM);
                break;
            case REV:
                topShooter.setPower(-0.5);
                bottomShooter.setPower(-0.5);
                break;
            case CLOSEAUTO:
                setShooterPIDPower(ShooterConstants.closeShootAutoRPM);
        }
    }

    public void setHood(double pos){
        hood.setPosition(pos);
    }
    public double ticksPerSecToRPM(double tps){
        return tps * 60.0 / ShooterConstants.TICKS_PER_REV;
    }
    public void setShooterPIDPower(double targetRPM){
        double topVelocity = Math.abs(topShooter.getVelocity());

        double currentRPM = (ticksPerSecToRPM(topVelocity));
        MyTelem.addData("Shooter Current RPM", currentRPM);
        shooterrpmPID.setPID(ShooterConstants.kp, ShooterConstants.ki, ShooterConstants.kd);
        double power = shooterrpmPID.calculate(currentRPM, targetRPM);
        power += (targetRPM > 0) ? (ShooterConstants.kf * (targetRPM / ShooterConstants.MAX_RPM)) : 0.0;
        power = Range.clip(power, 0, 1);
        double currentVoltage = Robot.voltage;
        MyTelem.addData("Power without voltage modifier", power);
        MyTelem.addData("Power with voltage modifier", power * 12.0 / currentVoltage);
        topShooter.setPower(power * 12.0 / currentVoltage);
        bottomShooter.setPower(power * 12.0 / currentVoltage);
    }


    public double CRticksPerSecToRPM(double tps){
        return tps * 60.0 / ShooterConstants.CR_TICKS_PER_REV;
    }

    public void setCounterRollerPIDPower(double targetRPM){
        double CRVelocity = Math.abs(counterRoller.getVelocity());

        double currentRPM = (CRticksPerSecToRPM(CRVelocity));
        MyTelem.addData("Counter roller Current RPM", currentRPM);
        counterRollerrpmPID.setPID(ShooterConstants.CRkp, ShooterConstants.CRki, ShooterConstants.CRkd);
        double power = counterRollerrpmPID.calculate(currentRPM, targetRPM);
        power += (targetRPM > 0) ? (ShooterConstants.CRkf * (targetRPM / ShooterConstants.MAX_RPM)) : 0.0;
        power = Range.clip(power, 0, 1);
        double currentVoltage = Robot.voltage;
        MyTelem.addData("Power without voltage modifier", power);
        MyTelem.addData("Power with voltage modifier", power * 12.0 / currentVoltage);
        counterRoller.setPower(power * 12.0 / currentVoltage);
    }


    public boolean atRPM(){
        return shooterrpmPID.atSetPoint();
    }

    public ShooterState getState(){
        return state;
    }

//    public enum ShooterState{
//        SHOOTING, STOP
//    }

    @Override
    public void periodic() {
        setState(state);
        shooterrpmPID.setPID(ShooterConstants.kp, ShooterConstants.ki, ShooterConstants.kd);
    }

    public enum ShooterState {
        CLOSE, FAR, STOP, TESTING, REV, CLOSEAUTO
    }
}
