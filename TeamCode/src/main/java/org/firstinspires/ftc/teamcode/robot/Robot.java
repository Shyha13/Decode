package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.MyTelem;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public class Robot {
    boolean auto = false;

    public Follower follower;

    // hardware stuff, servos, motors, etc.
    DcMotorEx backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor;
    DcMotorEx topShooterMotor, bottomShooterMotor, counterRoller;
    DcMotorEx intakeMotor;
    Servo hoodServo;
    Servo turretLeftServo, turretRightServo, blockerServo;
    CRServo kickerRightServo;

    // all subsystem classes
    public List<LynxModule> hubs;
    public Intake intake;
    public Shooter shooter;
    public Turret turret;
    public Kicker kicker;
    public Blocker blocker;

    public Pose currentPose;
    public boolean holding;
    public boolean red;
    public static double voltage = 12;
    private ElapsedTime timer;
    private double previousVoltageTime;
    public Robot(HardwareMap hm, boolean isAuto, String color){
        this(hm, isAuto);
        red = color.equals("RED");
    }
    public Robot (HardwareMap hm, boolean isAuto) {
        timer.reset();
        previousVoltageTime = timer.time(TimeUnit.MILLISECONDS);
        CommandScheduler.getInstance().reset();
        auto = isAuto;
        follower = new Follower(hm, FConstants.class, LConstants.class);
//        follower.breakFollowing();
        currentPose = AutoConstants.finalPose;
        follower.setStartingPose(currentPose);
        topShooterMotor = hm.get(DcMotorEx.class, "topShooter");
        bottomShooterMotor = hm.get(DcMotorEx.class, "bottomShooter");
        counterRoller = hm.get(DcMotorEx.class, "counterRoller");
        intakeMotor = hm.get(DcMotorEx.class, "intake");
        turretLeftServo = hm.get(Servo.class, "turretLeftServo");
        turretRightServo = hm.get(Servo.class, "turretRightServo");
        blockerServo = hm.get(Servo.class, "BlockerServo");
//        hoodServo = hm.get(Servo.class, "hoodServo");
//        kickerLeftServo = hm.get(CRServo.class, "kickerLeftServo");
        kickerRightServo = hm.get(CRServo.class, "kickerRightServo");
        //declare all hardware names
        topShooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bottomShooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        topShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bottomShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //ie. name = hm.get(Servo.class, "...");

        // We can in pedro for this as well.
        //        backLeft = hm.get(DcMotorEx.class, "backLeft");
        //        backRight = hm.get(DcMotorEx.class, "backRight");
        //        frontLeft = hm.get(DcMotorEx.class, "frontRight");
        //        frontRight = hm.get(DcMotorEx.class, "frontLeft");


        //handle auto specific behaviors
        //ie. resetting motor encoders, etc.


        //handle hardware specific settings
        //ie. zeropowermode, etc.

        //declare all subsystem objects
        intake = new Intake(intakeMotor);
        shooter = new Shooter(topShooterMotor, bottomShooterMotor, counterRoller, hoodServo);
        turret = new Turret(turretLeftServo, turretRightServo);
        kicker = new Kicker(kickerRightServo);
        blocker = new Blocker(blockerServo);

        //register subsystems
        CommandScheduler.getInstance().registerSubsystem(intake, shooter, turret); //kicker

        hubs = hm.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        voltage = hm.voltageSensor.iterator().next().getVoltage();
    }

    public void update(){
        CommandScheduler.getInstance().run();
        follower.update();

        if(intake != null)
            MyTelem.addData("Intake State", intake.getState());
        if(shooter != null)
            MyTelem.addData("Shooter State", shooter.getState());
        if(turret != null)
            MyTelem.addData("Turret State", turret.getState());
        if(kicker != null)
            MyTelem.addData("Kicker State", kicker.getState());

        for(LynxModule hub : hubs){
            hub.clearBulkCache();
        }

        if(!holding)
            currentPose = follower.getPose();

        if(timer.time(TimeUnit.MILLISECONDS) - previousVoltageTime > 1000){
            previousVoltageTime = timer.time(TimeUnit.MILLISECONDS);
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        }
        MyTelem.addData("Current Pose", currentPose);
        MyTelem.update();
    }

    public void stop(){
        CommandScheduler.getInstance().reset();

        for(LynxModule hub : hubs){
            hub.clearBulkCache();
        }
    }

    public void holding(){
        follower.holdPoint(currentPose);
        holding = true;
    }

    public void stopHolding(){
        follower.breakFollowing();
        follower.startTeleopDrive();
        follower.setMaxPower(1);
        holding = false;
    }
}
