package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants;

@Config
@TeleOp(name = "Shooter Tuner")
public class ShooterTuner extends LinearOpMode {
    public void runOpMode(){
        MyTelem.init(telemetry);
        Robot robot = new Robot(hardwareMap, false);
        robot.shooter.state = Shooter.ShooterState.TESTING;
        waitForStart();
        while (opModeIsActive()) {
//            robot.shooter.setPIDPower(ShooterConstants.tuningTestingRPM);
            MyTelem.addData("Shooter Target RPM", ShooterConstants.tuningTestingRPM);
            MyTelem.addData("Shooter At RPM", robot.shooter.atRPM());
            robot.update();
        }

        robot.stop();
    }
}
