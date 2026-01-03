package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants.RPM_OFFSET;
import static org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants.startingVelocity;
import static org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants.tuningTestingRPM;
import static org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants.currentVelocity;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants;

public class Shooter implements Subsystem {
    DcMotorEx shooterMotor, shooterMotor2;
    PIDController shooterRPMPID;

    public ShooterState state = ShooterState.STOP;
    public Shooter(DcMotorEx shooterMotor, DcMotorEx shooterMotor2){
        shooterRPMPID = new PIDController(ShooterConstants.kp, ShooterConstants.ki, ShooterConstants.kd);
        shooterRPMPID.setTolerance(10);

//        counterRollerPID = new PIDController(ShooterConstants.CRkp, ShooterConstants.CRki, ShooterConstants.CRkd);
//        counterRollerPID.setTolerance(10);
        this.shooterMotor = shooterMotor;
        this.shooterMotor2 = shooterMotor2;
    }

    public void setState(ShooterState state){
        this.state = state;
        switch (state) {
            case CLOSE:
                currentVelocity = ShooterConstants.closeShootRPM;
                break;
            case FAR:
                currentVelocity = ShooterConstants.farShootRPM;
                break;
            case STOP:
                currentVelocity = startingVelocity;
                break;
            case TESTING:
                currentVelocity = tuningTestingRPM;
                break;
            case SPEEDING_UP:
                currentVelocity = ShooterConstants.speedingVelocity;
                break;
            case MATH:
                double rpm = getRPM(Robot.currentPose);
                rpm = getRPM(Robot.getEffectiveCoordinates()) + RPM_OFFSET;
                currentVelocity = rpm;
                break;
        }
    }
    public double getRPM(Pose pose){
        Vector velocity = Robot.velocity;
        double distance = Robot.getDistanceFromGoalLL(pose);
        //Points:
        //



        //92.09: 1620, 3855.6
        //84.7: 1600, 3808
        //100.6: 1680, 3988.4
        //65.55: 1380, 3284.4
        //45.35: 1350, 3213
        //145.88: 2225, 5355
        //156.95: 2280, 5426.5
        //141.15: 2180, 5188.4
        //y=0.0385207x^{2}+1.30815x+1181.56736

       //kp=6 kf = 0.6
        //crkp=2 crkf=0.3
        //83.62, 2300 3
        //95.97, 2720 3
        //58.54, 1850 3
        //113.30, 2730 3
        //144.56, 3240 31
        //163.64, 3450 3
        //100.12, 2540 3
        //77.14, 2100 3

        //84.84, 2200 3
        //99.78, 2600 3
        //109.78, 2800
        //
        //3

        //84.21, 2500
        //104.91, 2710
        //58.01, 2325
        //79.37, 2485
        //67.15, 2445
        //110.69, 2825
        //90.07, 2550
        //96.75, 2580
        //134.8, 2965

        //84.803, 4120
        //108.96, 4350
        //117.46, 4400
        //60, 4000

        //y=0.0270551x^{2}+2.69484x+3513.8641
//        double speed = 1931.52586 * Math.pow(1.00321, distance) + RPM_OFFSET;
//        double speed = -0.0472686 * distance * distance + 25.9231 * distance + 500;
//        speed = -0.0472686 * distance * distance + 25.9231 * distance + 500;
        double speed = 0.0270551 * distance * distance + 2.69484 * distance + 3513.8641;
        MyTelem.addData("speed", speed);

        return speed;
    }

    public double ticksPerSecToRPM(double tps){
        return tps * 60.0 / ShooterConstants.TICKS_PER_REV;
    }
    public void setShooterPIDPower(double targetRPM){
        double topVelocity = Math.abs(shooterMotor.getVelocity());
        double currentRPM = (ticksPerSecToRPM(topVelocity));

        shooterRPMPID.setPID(ShooterConstants.kp, ShooterConstants.ki, ShooterConstants.kd);

        double power = shooterRPMPID.calculate(currentRPM, targetRPM);
        power += (targetRPM > 0) ? (ShooterConstants.kf * (targetRPM / ShooterConstants.MAX_RPM)) : 0.0;
        power = Range.clip(power, 0, 1);

        double currentVoltage = Robot.voltage;
        shooterMotor.setPower(power * 13 / currentVoltage);
        shooterMotor2.setPower(power * 13 / currentVoltage);

        MyTelem.addData("Shooter Current RPM", currentRPM);
        MyTelem.addData("Shooter Target RPM", targetRPM);
        MyTelem.addData("Shooter Power", power * 13 / currentVoltage);
    }
    public boolean shooterAtRPM(){
        return Math.abs(shooterRPMPID.getPositionError()) <= 200;
    }

    public boolean atRPM() {
        return shooterAtRPM();
    }

    public ShooterState getState(){
        return state;
    }

    @Override
    public void periodic() {
        setState(state);
        shooterRPMPID.setPID(ShooterConstants.kp, ShooterConstants.ki, ShooterConstants.kd);
        setShooterPIDPower(currentVelocity);
    }

    public enum ShooterState {
        CLOSE, FAR, STOP, TESTING, MATH, SPEEDING_UP
    }
}
