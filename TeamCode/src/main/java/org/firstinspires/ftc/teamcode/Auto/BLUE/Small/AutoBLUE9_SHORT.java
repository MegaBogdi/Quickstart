package org.firstinspires.ftc.teamcode.Auto.BLUE.Small;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.SharedUtils;
import org.firstinspires.ftc.teamcode.Auto.Utils;
import org.firstinspires.ftc.teamcode.Subsystems.IOSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
@Disabled
@Configurable
@Autonomous(name = "AutonomieBLUE_9_SHORT", group = "Auto RED")
public class AutoBLUE9_SHORT extends CommandOpMode {
    private Pose startPose = new Pose(110.729, 136.523, Math.toRadians(0));
    private Pose shootStartPos = new Pose(93.907, 84.776, Math.toRadians(0));
    private Pose spike1Pos = new Pose(126.056, 83.439, Math.toRadians(0));
    private Pose shootAfterSpike1Pos = new Pose(93.972, 83.822, Math.toRadians(0));
    private Pose shootToSpike2ControlPos = new Pose(93.112, 52.958, Math.toRadians(0));
    private Pose spike2Pos = new Pose(133.093, 58.916, Math.toRadians(0));
    private Pose spike2ToShootControlPos = new Pose(93.537, 52.640, Math.toRadians(0));
    private Pose shootAfterSpike2Pos = new Pose(93.757, 83.430, Math.toRadians(0));
    private Pose leverPos = new Pose(126.318, 74.019, Math.toRadians(0));
    private Pose parkPos = new Pose(118.804, 91.486, Math.toRadians(90));

    private PathChain startToShootPath;
    private PathChain shootToSpike1Path;
    private PathChain spike1ToShootPath;
    private PathChain shootToSpike2Path;
    private PathChain spike2ToShootPath;
    private PathChain shootToLeverPath;
    private PathChain leverToParkPath;

    private List<LynxModule> hubs;

    private TelemetryManager telemetryA;
    private IOSubsystem IO;
    private Follower follower;

    private Utils utils;
    private double alpha;
    private boolean succesDemand;
    private int id;

    private ElapsedTime timer;

    private int autoOutakeTimeout = 5000;

    @Override
    public void initialize() {
        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setPose(startPose);
        follower.setMaxPower(1);

        IO = new IOSubsystem(hardwareMap);
        register(IO);
        IO.startLimeLight();

        utils = new Utils(IO, follower);

        telemetryA = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryA.update(telemetry);

        IO.teamIsRed = true;

        timer = new ElapsedTime();
        timer.startTime();

        startToShootPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        startPose,
                        shootStartPos
                ))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        shootToSpike1Path = follower.pathBuilder()
                .addPath(new BezierLine(
                        shootStartPos,
                        spike1Pos
                ))
                .setConstantHeadingInterpolation(shootStartPos.getHeading())
                .build();

        spike1ToShootPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        spike1Pos,
                        shootAfterSpike1Pos
                ))
                .setConstantHeadingInterpolation(spike1Pos.getHeading())
                .build();

        shootToSpike2Path = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootAfterSpike1Pos,
                        shootToSpike2ControlPos,
                        spike2Pos
                ))
                .setConstantHeadingInterpolation(shootAfterSpike1Pos.getHeading())
                .build();

        spike2ToShootPath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        spike2Pos,
                        spike2ToShootControlPos,
                        shootAfterSpike2Pos
                ))
                .setConstantHeadingInterpolation(spike2Pos.getHeading())
                .build();

        shootToLeverPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        shootAfterSpike2Pos,
                        leverPos
                ))
                .setConstantHeadingInterpolation(shootAfterSpike2Pos.getHeading())
                .build();

        leverToParkPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        leverPos,
                        parkPos
                ))
                .setLinearHeadingInterpolation(leverPos.getHeading(), parkPos.getHeading())
                .build();





        schedule(
                new RunCommand(() -> {
                    if (IO.jam && !IO.recovering) {
                        IO.recovering = true;
                        schedule(utils.createRecoverySeq());
                    }
                }),
                new SequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        new InstantCommand(() -> {
                            IO.ALL[0] = 1;
                            IO.ALL[1] = 2;
                            IO.ALL[2] = 1;
                        }),
                        new FollowPathCommand(follower, startToShootPath)
                                .beforeStarting(() -> follower.setMaxPower(1)),
                        new ParallelCommandGroup(
                                utils.newMotify(1000),
                                new InstantCommand(() -> IO.setTargetTurretRads(Math.toRadians(-90)))
                        ),
                        utils.newAutoOutake(4500),
                        new FollowPathCommand(follower, shootToSpike1Path)
                                .beforeStarting(()->IO.setLed(1))
                                .beforeStarting(() -> follower.setMaxPower(0.55))
                                .alongWith(utils.newStartIntake(4000))
                                .andThen(utils.newStopIntake())
                                .andThen(new InstantCommand(()->{if (IO.ocupied()>=3){IO.setLed(0.5);} else {IO.setLed(0.277);}})),
                        new FollowPathCommand(follower, spike1ToShootPath)
                                .beforeStarting(() -> follower.setMaxPower(1)),
                        utils.newAutoOutake(3000),
                        new FollowPathCommand(follower, shootToSpike2Path)
                                .beforeStarting(()->IO.setLed(1))
                                .alongWith(utils.newStartIntake(4000))
                                .beforeStarting(() -> follower.setMaxPower(0.45))
                                .andThen(utils.newStopIntake())
                                .andThen(new InstantCommand(()->{if (IO.ocupied()>=3){IO.setLed(0.5);} else {IO.setLed(0.277);}})),
                        new FollowPathCommand(follower, spike2ToShootPath)
                                .beforeStarting(() -> follower.setMaxPower(1)),
                        utils.newAutoOutake(3000),
                        new FollowPathCommand(follower, shootToLeverPath)
                                .beforeStarting(() -> follower.setMaxPower(1)),
                        new WaitCommand(1500),
                        new FollowPathCommand(follower, leverToParkPath)
                                .beforeStarting(() -> follower.setMaxPower(1)),
                        new InstantCommand(()->IO.setTargetTurretRads(0)),
                        new InstantCommand(() -> {SharedUtils.sharedPose = follower.getPose(); IO.setLed(0.722);})
                )
        );
    }

    @Override
    public void run() {
        hubs.forEach(LynxModule::clearBulkCache);

        super.run();

        if (!opModeIsActive() || isStopRequested()) {
            SharedUtils.sharedPose = follower.getPose();
        }

        follower.update();

        Pose p = follower.getPose();

        telemetryA.debug("Shared", SharedUtils.sharedPose == null ? "no" : SharedUtils.sharedPose.getHeading());
        telemetryA.debug("dist: " + IO.getDistanceOdom(follower.getPose()));
        telemetryA.debug(String.format("coord: x=%.2f  y=%.2f  h=%.1f", p.getX(), p.getY(), Math.toDegrees(p.getHeading())));
        telemetryA.debug(String.format("ALL: 0=%d | 1=%d | 2=%d", IO.ALL[0], IO.ALL[1], IO.ALL[2]));
        telemetryA.debug(String.format("condition: RPM:%b | turret:%b | sorter:%b", IO.isRPMready(), IO.isTurretReady(alpha, follower), IO.isSorterReady()));
        telemetryA.debug("rpm: " + IO.returnRPM());
        telemetryA.debug("targetRpm: " + IO.returnTargetRPM());
        telemetryA.update(telemetry);

        IO.update_sep_pid(timer.seconds());
        IO.update_RPM_pid();
        IO.update_turret_pid();
    }

}
