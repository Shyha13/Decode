package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
@Config
@TeleOp(name = "CR Servo Tester")
public class CRServoTester extends LinearOpMode {
    public static String servoName = "";
    public static double power = 0.1;

    public void runOpMode(){
        CRServo servo = hardwareMap.get(CRServo.class, servoName);
        waitForStart();
        while(opModeIsActive()){
            servo.setPower(power);
        }
    }
}
