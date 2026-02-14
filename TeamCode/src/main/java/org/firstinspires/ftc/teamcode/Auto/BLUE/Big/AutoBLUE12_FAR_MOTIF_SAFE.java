package org.firstinspires.ftc.teamcode.Auto.BLUE.Big;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.SharedUtils;
import org.firstinspires.ftc.teamcode.Auto.Utils;
import org.firstinspires.ftc.teamcode.Subsystems.IOSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Configurable
@Autonomous(name = "AutonomieBLUE_12_FAR_MOTIF_SAFE", group = "Auto Blue")
public class AutoBLUE12_FAR_MOTIF_SAFE extends CommandOpMode {
    // Pedro visualizer poses (mapped to a similar route structure)
    private Pose startPose = new Pose(59.019, 6.857, Math.toRadians(180));
    private Pose spike2Pos = new Pose(11.373831775700934, 59.934579439252346, Math.toRadians(180));   // y =62
    private Pose spike2LeverPos = new Pose(19.19532710280374, 65.49532710280373, Math.toRadians(180));
    private Pose shootFarPos = new Pose(59.701, 19.570, Math.toRadians(180));
    private Pose spike3Pos = new Pose(10.477, 33.804, Math.toRadians(180));
    private Pose shootShortPos = new Pose(50.393, 83.290, Math.toRadians(180));
    private Pose spike1Pos = new Pose(18.224, 83.280, Math.toRadians(180));
    private Pose shootShortReturnPos = new Pose(50.439, 83.252, Math.toRadians(180));
    private Pose parkPos = new Pose(50.299, 68.178, Math.toRadians(90));

    private PathChain startToSpike2Path;
    private PathChain spike2ToLeverPath;
    private PathChain leverToShootFarPath;
    private PathChain shootFarToSpike3Path;
    private PathChain spike3ToShootShortPath;
    private PathChain shootShortToSpike1Path;
    private PathChain spike1ToShootShortPath;
    private PathChain shootShortToParkPath;

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

        IO.teamIsRed = false;

        timer = new ElapsedTime();
        timer.startTime();

        startToSpike2Path = follower.pathBuilder()
                .addPath(new BezierCurve(
                        startPose,
                        new Pose(68.56074766355142,72.83504672897199),
                        spike2Pos))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStrength(1)
                .setBrakingStart(1)
                .build();

        spike2ToLeverPath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        spike2Pos,
                        new Pose(27.140186915887842, 59.78504672897198),
                        spike2LeverPos
                ))
                .setBrakingStrength(1)
                .setBrakingStart(1)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        leverToShootFarPath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        spike2LeverPos,
                        new Pose(47.126, 51.089),
                        shootFarPos
                ))
                .setBrakingStrength(0.7)
                .setBrakingStart(0.75)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shootFarToSpike3Path = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootFarPos,
                        new Pose(52.136, 38.360),
                        spike3Pos
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStrength(1)
                .setBrakingStart(1)
                .build();

        spike3ToShootShortPath = follower.pathBuilder()
                .addPath(new BezierLine(spike3Pos, shootShortPos))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStrength(1)
                .setBrakingStart(1)
                .build();

        shootShortToSpike1Path = follower.pathBuilder()
                .addPath(new BezierLine(shootShortPos, spike1Pos))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        spike1ToShootShortPath = follower.pathBuilder()
                .addPath(new BezierLine(spike1Pos, shootShortReturnPos))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStrength(1)
                .setBrakingStart(1)
                .build();

        shootShortToParkPath = follower.pathBuilder()
                .addPath(new BezierLine(shootShortReturnPos, parkPos))
                .setLinearHeadingInterpolation(Math.toRadians(180), parkPos.getHeading())
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
                                new InstantCommand(() -> IO.setTargetTurretRads(Math.toRadians(90)))
                        ),
                        utils.newAutoOutake(4500),
                        new FollowPathCommand(follower, startToSpike2Path)
                                .beforeStarting(()->IO.setLed(1))
                                .beforeStarting(() -> follower.setMaxPower(0.55))
                                .alongWith(utils.newStartIntake(4000))
                                .andThen(utils.newStopIntake())
                                .andThen(new InstantCommand(()->{if (IO.ocupied()>=3){IO.setLed(0.5);} else {IO.setLed(0.277);}})),

                        new FollowPathCommand(follower, spike2ToLeverPath)
                                .beforeStarting(() -> follower.setMaxPower(1)),
                        new WaitCommand(500),
                        new FollowPathCommand(follower, leverToShootFarPath)
                                .beforeStarting(() -> follower.setMaxPower(1)),
                        utils.newAutoOutake(4500),
                        new FollowPathCommand(follower, shootFarToSpike3Path)
                                .beforeStarting(()->IO.setLed(1))
                                .alongWith(utils.newStartIntake(5000))
                                .beforeStarting(() -> follower.setMaxPower(0.45))
                                .andThen(utils.newStopIntake())
                                .andThen(new InstantCommand(()->{if (IO.ocupied()>=3){IO.setLed(0.5);} else {IO.setLed(0.277);}})),
                        new FollowPathCommand(follower, spike3ToShootShortPath)
                                .beforeStarting(() -> follower.setMaxPower(1)),
                        utils.newAutoOutake(3000),
                        new FollowPathCommand(follower, shootShortToSpike1Path)
                                .beforeStarting(()->IO.setLed(1))
                                .beforeStarting(() -> follower.setMaxPower(0.5))
                                .alongWith(utils.newStartIntake(3500))
                                .andThen(new InstantCommand(()->{if (IO.ocupied()>=3){IO.setLed(0.5);} else {IO.setLed(0.277);}}))
                                .andThen(utils.newStopIntake()),
                        new FollowPathCommand(follower, spike1ToShootShortPath)
                                .beforeStarting(() -> follower.setMaxPower(1)),
                        new WaitCommand(100),
                        utils.newAutoOutake(3000),
                        new InstantCommand(()->IO.setTargetTurretRads(0)),
                        new FollowPathCommand(follower, shootShortToParkPath)
                                .beforeStarting(() -> follower.setMaxPower(1)),
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
