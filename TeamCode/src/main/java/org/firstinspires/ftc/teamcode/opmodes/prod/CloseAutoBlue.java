package org.firstinspires.ftc.teamcode.opmodes.prod;

import static org.firstinspires.ftc.teamcode.pedroPathing.constants.AutoConstants.firstIntakeX;
import static org.firstinspires.ftc.teamcode.pedroPathing.constants.AutoConstants.firstWait;
import static org.firstinspires.ftc.teamcode.pedroPathing.constants.AutoConstants.intakeBallWait;
import static org.firstinspires.ftc.teamcode.pedroPathing.constants.AutoConstants.secondIntakeX;
import static org.firstinspires.ftc.teamcode.pedroPathing.constants.AutoConstants.secondWait;
import static org.firstinspires.ftc.teamcode.pedroPathing.constants.AutoConstants.shootingX;
import static org.firstinspires.ftc.teamcode.pedroPathing.constants.AutoConstants.shootingY;
import static org.firstinspires.ftc.teamcode.pedroPathing.constants.AutoConstants.thirdWait;

import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.BlockerCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.KickerCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.ShooterCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.TurretCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.MyTelem;

@Autonomous(name = "Close Side Auto Blue", group = "Prod")
@Config
public class CloseAutoBlue extends OpMode {

    public static Pose autoEndPose;
    private ElapsedTime timer;
    private DashboardPoseTracker dashboardPoseTracker;
    private Robot robot;
    private CloseSideAutoPaths paths;

    // 🔥 REQUIRED FIX: This stores the full auto routine
    private SequentialCommandGroup auto;

    @Override
    public void init() {
        timer = new ElapsedTime();
        MyTelem.init(telemetry);

        robot = new Robot(hardwareMap, true);

        // Preloading initialization commands
        CommandScheduler.getInstance().schedule(
                new BlockerCommand(robot, Blocker.BlockerState.BLOCKED),
                new TurretCommand(robot, Turret.TurretState.FRONT)
        );

        // Transfer sequence
        CommandGroupBase transferCommand = new SequentialCommandGroup(
                new IntakeCommand(robot, Intake.IntakeState.OFF),
                new BlockerCommand(robot, Blocker.BlockerState.UNBLOCKED),
                new WaitCommand(firstWait),
                new KickerCommand(robot, Kicker.KickerState.ON),
                new WaitCommand(secondWait),
                new IntakeCommand(robot, Intake.IntakeState.ON),
                new WaitCommand(thirdWait),
                new ShooterCommand(robot, Shooter.ShooterState.STOP),
                new KickerCommand(robot, Kicker.KickerState.OFF),
                new BlockerCommand(robot, Blocker.BlockerState.BLOCKED)
        );

        CommandGroupBase shootThree = new SequentialCommandGroup(transferCommand);

        paths = new CloseSideAutoPaths(robot.follower);

        // 🔥 FIX: STORE AUTO AS FIELD
        auto = new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, paths.Path1),
                new ShooterCommand(robot, Shooter.ShooterState.CLOSEAUTO),
                shootThree,

                new FollowPathCommand(robot.follower, paths.Path2),
                new WaitCommand(intakeBallWait),
                new FollowPathCommand(robot.follower, paths.Path3),
                new ShooterCommand(robot, Shooter.ShooterState.CLOSEAUTO),
                shootThree,

                new FollowPathCommand(robot.follower, paths.Path4),
                new WaitCommand(intakeBallWait),
                new FollowPathCommand(robot.follower, paths.Path5),
                new ShooterCommand(robot, Shooter.ShooterState.CLOSEAUTO),
                shootThree,

                new FollowPathCommand(robot.follower, paths.Path6),
                new WaitCommand(intakeBallWait),
                new FollowPathCommand(robot.follower, paths.Path7),
                new ShooterCommand(robot, Shooter.ShooterState.CLOSEAUTO),
                shootThree,

                // PARK PATH
                new FollowPathCommand(
                        robot.follower,
                        robot.follower.pathBuilder()
                                .addPath(new BezierLine(
                                        new Pose(robot.follower.getPose().getX(), robot.follower.getPose().getY()),
                                        new Pose(51.9627, 70.1784)
                                ))
                                .setConstantHeadingInterpolation(robot.follower.getPose().getHeading())
                                .build()
                )
        );

        dashboardPoseTracker = new DashboardPoseTracker(robot.follower.poseUpdater);
        Drawing.drawRobot(robot.follower.poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    @Override
    public void loop() {

        // 🔥🔥 REQUIRED FIX: This makes commands run!!!
        CommandScheduler.getInstance().run();

        robot.update();

        MyTelem.addData("POSE", robot.follower.getPose());
        MyTelem.addData("TIMER", timer.seconds());
        MyTelem.update();

        dashboardPoseTracker.update();
        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(robot.follower.poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();

        Robot.currentPose = robot.follower.getPose();
    }

    @Override
    public void start() {
        // 🔥 FIX: Actually schedule the auto routine
        CommandScheduler.getInstance().schedule(auto);
    }

    public static class CloseSideAutoPaths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7;

        public CloseSideAutoPaths(Follower follower) {

            follower.setStartingPose(new Pose(19.250, 123.000, Math.toRadians(143)));

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(19.250, 123.000), new Pose(shootingX, shootingY)))
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(AutoConstants.shootingAngle))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(shootingX, shootingY),
                            new Pose(56.86, 78.80),
                            new Pose(firstIntakeX, 84.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(AutoConstants.shootingAngle), Math.toRadians(180))
                    .setZeroPowerAccelerationMultiplier(2)
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(firstIntakeX, 84.000), new Pose(shootingX, shootingY)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(AutoConstants.shootingAngle))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(shootingX, shootingY),
                            new Pose(66, 52),
                            new Pose(secondIntakeX, 60.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(AutoConstants.shootingAngle), Math.toRadians(180))
                    .setZeroPowerAccelerationMultiplier(2)
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(secondIntakeX, 60.000), new Pose(shootingX, shootingY)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(AutoConstants.shootingAngle))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(shootingX, shootingY),
                            new Pose(87, 31),
                            new Pose(secondIntakeX, 36.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(AutoConstants.shootingAngle), Math.toRadians(180))
                    .setZeroPowerAccelerationMultiplier(2)
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(secondIntakeX, 36.000), new Pose(shootingX, shootingY)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(AutoConstants.shootingAngle))
                    .build();
        }
    }

    @Override

    public void stop() {
        Pose finalPose = robot.follower.getPose();
    }

}
