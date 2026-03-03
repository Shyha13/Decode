package org.firstinspires.ftc.teamcode.opmodes.prod;

import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.intakeHead;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.lvHead;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.s1Head;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.s2Head;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants.shootingAngle;
import static org.firstinspires.ftc.teamcode.utils.constants.auto.CloseSideAutoPoseData.lever;

import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.TransferCancelCommand;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.TransferCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.TurretCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.utils.constants.auto.AutoConstants;
import org.firstinspires.ftc.teamcode.utils.constants.auto.CloseSideAutoPoseData;

public class ClosePath extends OpMode {
    private ElapsedTime timer;
    private DashboardPoseTracker dashboardPoseTracker;
    private Robot robot;
    private CloseSideAutoPaths paths;
    private SequentialCommandGroup auto;
    private String color;

    public ClosePath(String color) {
        this.color = color;
    }

    @Override
    public void init() {
        timer = new ElapsedTime();
        MyTelem.init(telemetry);
        robot = new Robot(hardwareMap, true, color);
        paths = new CloseSideAutoPaths(robot.follower, color);

      /*  CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new BlockerCommand(robot, Blocker.BlockerState.BLOCKED),
                        new TurretCommand(robot, Turret.TurretState.MATH)
                )
        );

       */


        auto = new SequentialCommandGroup(
                new TurretCommand(robot, Turret.TurretState.MATH),
                new IntakeCommand(robot, Intake.IntakeState.ON),
                new FollowPathCommand(robot.follower, paths.preload),
            //    shootThree(),
                new IntakeCommand(robot, Intake.IntakeState.ON),
                new FollowPathCommand(robot.follower, paths.spike1),
                new WaitCommand(500),
                new FollowPathCommand(robot.follower, paths.shootInt),
          //      shootThree(),
                new WaitCommand(300),
                new FollowPathCommand(robot.follower, paths.spike2),
                new FollowPathCommand(robot.follower, paths.return2),
                //shootThree(),
                new WaitCommand(750),
                new FollowPathCommand(robot.follower, paths.LeverPath),
                new WaitCommand(50),
                new FollowPathCommand(robot.follower, paths.LeverIntakePath),
                new IntakeCommand(robot, Intake.IntakeState.ON),
                new WaitCommand(500),
                new FollowPathCommand(robot.follower, paths.LeverReturnPath),
                //shootThree,
                new IntakeCommand(robot, Intake.IntakeState.ON),
                new FollowPathCommand(robot.follower, paths.LeverPath2),
                new WaitCommand(50),
                new FollowPathCommand(robot.follower, paths.LeverIntakePath2),
                new IntakeCommand(robot, Intake.IntakeState.ON),
                new WaitCommand(800),
                new ParallelCommandGroup(
                        new FollowPathCommand(robot.follower, paths.LeverReturnPath2),
                        new IntakeCommand(robot, Intake.IntakeState.ON)
                ),
//                shootThree(),
                new IntakeCommand(robot, Intake.IntakeState.ON)
//                new ParallelCommandGroup(
//                        new FollowPathCommand(robot.follower, paths.spike1)
//                ),
//                new ParallelCommandGroup(
//                        new FollowPathCommand(robot.follower, paths.return1),
//                        new IntakeCommand(robot, Intake.IntakeState.ON)
//                ),
//                //shootThree(),
//                new FollowPathCommand(robot.follower, paths.spike3),
//                new IntakeCommand(robot, Intake.IntakeState.SOLOFRONT),
//                new FollowPathCommand(robot.follower, paths.return3),
//                new InstantCommand(() -> ShooterConstants.karthikstfu = true)
        );
            //    new TransferCommand(robot)

        dashboardPoseTracker = new DashboardPoseTracker(robot.follower.poseUpdater);
        Drawing.drawRobot(robot.follower.poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    private CommandGroupBase shootThree() {
        return new SequentialCommandGroup(
                new TransferCommand(robot),
                new TransferCancelCommand(robot, Shooter.ShooterState.SPEEDING_UP),
                new IntakeCommand(robot, Intake.IntakeState.ON)
        );
    }

    @Override
    public void loop() {
        MyTelem.addData("POSE", robot.follower.getPose());
        MyTelem.addData("TIMER", timer.seconds());
        robot.update();
        dashboardPoseTracker.update();
        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(robot.follower.poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    @Override
    public void start() {
        timer.reset();
        CommandScheduler.getInstance().schedule(
                auto
        );
    }

    public static class CloseSideAutoPaths {
        public PathChain preload, shootInt, spike2, shoot, spike1, return1, return2, spike3, return3;
        public PathChain LeverPath, LeverPath2, LeverIntakePath, LeverReturnPath, LeverIntakePath2, LeverReturnPath2;

        public CloseSideAutoPaths(Follower follower, String color) {
            Pose startPos = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.startPose, color);
            Pose shootIntPos = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.shootInt, color);
            Pose shoot1Pos = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.shoot1, color);
            Pose shootPos = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.shoot, color);
            Pose s1Pos = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.s1, color);
            Pose s2Con = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.s2Con, color);
            Pose s2Pos = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.s2, color);
            Pose s3Con = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.s3Con, color);
            Pose s3Pos = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.s3, color);
            Pose leverPos = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.lever, color);
            Pose leverIntakePose = CloseSideAutoPoseData.mirror(CloseSideAutoPoseData.LEVER_INTAKE, color);


            double heading180 = CloseSideAutoPoseData.mirrorHeading(180, color);

            double finalShootHeading = CloseSideAutoPoseData.mirrorHeading(AutoConstants.finalShootHeading, color);
            follower.setStartingPose(startPos);

            preload = follower.pathBuilder()
                    .addPath(new BezierLine(startPos, shoot1Pos))
                    .setTangentHeadingInterpolation()
                    .setZeroPowerAccelerationMultiplier(3)
                    .setReversed(false)
                    .build();
            spike1 = follower.pathBuilder()
                    .addPath(new BezierLine(shoot1Pos, s1Pos))
                    .setLinearHeadingInterpolation(Math.toRadians(270), s1Head)
                    .setZeroPowerAccelerationMultiplier(2)
                    .build();

            shootInt = follower.pathBuilder()
                    .addPath(new BezierLine(s1Pos, shoot1Pos))
                    .setLinearHeadingInterpolation(s1Head, Math.toRadians(180))
                    .setReversed(true)
                    .build();

            return2 = follower.pathBuilder()
                    .addPath(new BezierLine(s2Pos, shootPos))
                    .setTangentHeadingInterpolation()
                    .setReversed(true)
                    .build();

            shoot = follower.pathBuilder()
                    .addPath(new BezierLine(s2Pos, shootPos))
                    .setLinearHeadingInterpolation(s1Head, Math.toRadians(180))
                    .setReversed(true)
                    .build();
            LeverPath = follower.pathBuilder()
                    .addPath(new BezierLine(shootPos, leverPos))
                    .setLinearHeadingInterpolation(Math.toRadians(shootingAngle), Math.toRadians(lvHead))
                    .setZeroPowerAccelerationMultiplier(3)
                    .build();
            LeverIntakePath = follower.pathBuilder()
                    .addPath(new BezierLine(leverPos, leverIntakePose))
                    .setLinearHeadingInterpolation(Math.toRadians(lvHead), Math.toRadians(intakeHead))
                    .build();
            LeverReturnPath = follower.pathBuilder()
                    .addPath(new BezierLine(leverIntakePose, shootPos))
                    .setTangentHeadingInterpolation()
                    .setZeroPowerAccelerationMultiplier(3)
                    .setReversed(true)
                    .build();

            LeverPath2 = follower.pathBuilder()
                    .addPath(new BezierLine(shootPos, leverPos))
                    .setLinearHeadingInterpolation(Math.toRadians(shootingAngle), Math.toRadians(lvHead))
                    .setZeroPowerAccelerationMultiplier(3)
                    .build();
            LeverIntakePath2 = follower.pathBuilder()
                    .addPath(new BezierLine(leverPos, leverIntakePose))
                    .setLinearHeadingInterpolation(Math.toRadians(lvHead), Math.toRadians(intakeHead))
                    .build();
            LeverReturnPath2 = follower.pathBuilder()
                    .addPath(new BezierLine(leverIntakePose, shootPos))
                    .setTangentHeadingInterpolation()
                    .setZeroPowerAccelerationMultiplier(3)
                    .setReversed(true)
                    .build();

            spike2 = follower.pathBuilder()
                    .addPath(new BezierCurve(shoot1Pos, s2Pos))
                    .setTangentHeadingInterpolation()
                    .setReversed(false)
                    .setZeroPowerAccelerationMultiplier(5)
//                    .setLinearHeadingInterpolation(Math.toRadians(heading180), Math.toRadians(heading180))
                    .build();
            return1 = follower.pathBuilder()
                    .addPath(new BezierLine(s2Pos, shootPos))
                    .setTangentHeadingInterpolation()
                    .setZeroPowerAccelerationMultiplier(3)
                    .setReversed(true)
                    .build();
            spike3 = follower.pathBuilder()
                    .addPath(new BezierCurve(shootPos, s3Con, s3Pos))
                    .setLinearHeadingInterpolation(Math.toRadians(finalShootHeading), Math.toRadians(heading180))
                    .build();
            return3 = follower.pathBuilder()
                    .addPath(new BezierLine(s3Pos, shootPos))
                    .setTangentHeadingInterpolation()
                    .setReversed(true)
                    .setZeroPowerAccelerationMultiplier(5)
                    .build();
        }
    }
}
