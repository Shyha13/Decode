package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.MyTelem;

@Config
@TeleOp(name = "Hood Tester")
public class HoodAndShooterTester extends LinearOpMode{
    @Override
    public void runOpMode(){
        MyTelem.init(telemetry);
        Robot robot = new Robot(hardwareMap, false);

        waitForStart();

        while (opModeIsActive()) {
            robot.shooter.state = Shooter.ShooterState.TESTING;
            MyTelem.addLine("--- SHOOTER ---");
            MyTelem.addData("At RPM", robot.shooter.shooterAtRPM());
            MyTelem.addLine();
            robot.update();
        }
    }
}
