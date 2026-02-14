package org.firstinspires.ftc.teamcode.Auto.BLUE.Big;

import com.arcrobotics.ftclib.command.RunCommand;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.SharedUtils;
import org.firstinspires.ftc.teamcode.Auto.Utils;
import org.firstinspires.ftc.teamcode.Subsystems.IOSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled
@Configurable
@Autonomous(name = "AutonomieBLUE_12_FAR", group = "Auto Blue")
public class AutoBLUE12_FAR extends CommandOpMode {
    // Pedro visualizer poses (mapped to the same route structure names)
    private Pose startPose = new Pose(57.019, 6.857, Math.toRadians(180));
    private Pose spike3Pos = new Pose(12.000, 36.254, Math.toRadians(180));
    private Pose helpSpike3Pos = new Pose(58.438, 49.425, Math.toRadians(180));
    private Pose shootFarPos = new Pose(57.150, 14.178, Math.toRadians(180));
    private Pose spike2Pos = new Pose(10.790, 59.860, Math.toRadians(180));
    private Pose helpSpike2Pos = new Pose(59.089, 70.766, Math.toRadians(180));
    private Pose helpSpike2ReturnPos = new Pose(49.224, 66.528, Math.toRadians(180));
    private Pose shootShortPos = new Pose(51.318, 84.187, Math.toRadians(180));
    private Pose spike1Pos = new Pose(17.140, 83.720, Math.toRadians(180));
    private Pose spike1ExitPos = new Pose(24.056, 76.879, Math.toRadians(180));
    private Pose spike1TransitPos = new Pose(17.234, 69.953, Math.toRadians(180));
    private Pose parkPos = new Pose(50.421, 64.262, Math.toRadians(90));

    private PathChain StartToSpike3Path;
    private PathChain Spike3ToShootFarPath;
    private PathChain ShootFarToSpike2LeverPath;
    private PathChain LeverToShootShortPath;
    private PathChain ShootShortToSpike1Path;
    private PathChain Spike1ToSpike1ExitPath;
    private PathChain Spike1ExitToSpike1TransitPath;
    private PathChain Spike1TransitToShootShortPath;
    private PathChain ShootShortToParkPath;

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

        IO.teamIsRed = false;

        timer = new ElapsedTime();
        timer.startTime();

        StartToSpike3Path = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, helpSpike3Pos, spike3Pos))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStrength(0.9)
                .setBrakingStart(0.75)
                .build();

        Spike3ToShootFarPath = follower.pathBuilder()
                .addPath(new BezierLine(spike3Pos, shootFarPos))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        ShootFarToSpike2LeverPath = follower.pathBuilder()
                .addPath(new BezierCurve(shootFarPos, helpSpike2Pos, spike2Pos))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        LeverToShootShortPath = follower.pathBuilder()
                .addPath(new BezierCurve(spike2Pos, helpSpike2ReturnPos, shootShortPos))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        ShootShortToSpike1Path = follower.pathBuilder()
                .addPath(new BezierLine(shootShortPos, spike1Pos))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStrength(0.7)
                .setBrakingStart(0.6)
                .build();

        Spike1ToSpike1ExitPath = follower.pathBuilder()
                .addPath(new BezierLine(spike1Pos, spike1ExitPos))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Spike1ExitToSpike1TransitPath = follower.pathBuilder()
                .addPath(new BezierLine(spike1ExitPos, spike1TransitPos))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Spike1TransitToShootShortPath = follower.pathBuilder()
                .addPath(new BezierLine(spike1TransitPos, shootShortPos))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStart(1)
                .setBrakingStrength(1)
                .build();

        ShootShortToParkPath = follower.pathBuilder()
                .addPath(new BezierLine(shootShortPos, parkPos))
                .setLinearHeadingInterpolation(shootShortPos.getHeading(), parkPos.getHeading())
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
                        new ParallelCommandGroup(
                                utils.newMotify(1000),
                                new InstantCommand(() -> IO.setTargetTurretRads(Math.toRadians(-90)))
                        ),
                        utils.newAutoOutake(4500),
                        new FollowPathCommand(follower, StartToSpike3Path)
                                .beforeStarting(() -> follower.setMaxPower(0.45))
                                .alongWith(utils.newStartIntake(5000))
                                .andThen(utils.newStopIntake()),
                        new WaitCommand(300),
                        new FollowPathCommand(follower, Spike3ToShootFarPath)
                                .beforeStarting(() -> follower.setMaxPower(0.7)),
                        utils.newAutoOutake(4500),
                        new FollowPathCommand(follower, ShootFarToSpike2LeverPath)
                                .alongWith(utils.newStartIntake(4000))
                                .beforeStarting(() -> follower.setMaxPower(0.7)),
                        new WaitCommand(750),
                        new FollowPathCommand(follower, LeverToShootShortPath)
                                .beforeStarting(() -> follower.setMaxPower(1)),
                        utils.newAutoOutake(3000),
                        new FollowPathCommand(follower, ShootShortToSpike1Path)
                                .beforeStarting(() -> follower.setMaxPower(0.8))
                                .alongWith(utils.newStartIntake(3500))
                                .andThen(utils.newStopIntake()),
                        new FollowPathCommand(follower, Spike1ToSpike1ExitPath)
                                .beforeStarting(() -> follower.setMaxPower(0.8)),
                        new FollowPathCommand(follower, Spike1ExitToSpike1TransitPath)
                                .beforeStarting(() -> follower.setMaxPower(0.8)),
                        new FollowPathCommand(follower, Spike1TransitToShootShortPath)
                                .beforeStarting(() -> follower.setMaxPower(1)),
                        new WaitCommand(100),
                        utils.newAutoOutake(3000),
                        new FollowPathCommand(follower, ShootShortToParkPath)
                                .beforeStarting(() -> follower.setMaxPower(1)),
                        new InstantCommand(() -> SharedUtils.sharedPose = follower.getPose())
                )
        );
    }

    @Override
    public void run() {
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
        telemetryA.debug(String.format("condition: RPM:%b | turret:%b,sorter:%b", IO.isRPMready(), IO.isTurretReady(alpha, follower), IO.isSorterReady()));
        telemetryA.debug("rpm: " + IO.returnRPM());
        telemetryA.debug("targetRpm: " + IO.returnTargetRPM());
        telemetryA.debug("motify: " + IO.getMotif());
        telemetryA.update(telemetry);

        IO.update_sep_pid(timer.seconds());
        IO.update_RPM_pid();
        IO.update_turret_pid();
    }
}
/* uai deci:

* nu e bun demand intexu pentru ca isi ia offset cand nu are ce ii trebuie
-fa un demand_index_offset




*/
