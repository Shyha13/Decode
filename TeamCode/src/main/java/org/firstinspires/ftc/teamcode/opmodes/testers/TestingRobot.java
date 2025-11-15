package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.MyTelem;

@TeleOp
public class TestingRobot extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        MyTelem.init(telemetry);
        robot = new Robot(hardwareMap, false);

        waitForStart();

        while(opModeIsActive()){
            MyTelem.addData("HELLO WORLD", true);
            robot.update();
        }
        robot.stop();
    }
}
