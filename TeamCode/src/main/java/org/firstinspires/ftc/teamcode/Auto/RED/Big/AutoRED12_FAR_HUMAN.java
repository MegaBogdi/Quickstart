package org.firstinspires.ftc.teamcode.Auto.RED.Big;

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
@Autonomous(name = "AutonomieRED_12_FAR_HUMAN", group = "Auto Red")
public class AutoRED12_FAR_HUMAN extends CommandOpMode {
    // Pedro visualizer poses
    private Pose startPose = new Pose(59.019, 6.857, Math.toRadians(180));
    private Pose preSpike2Pos = new Pose(11.374, 59.935, Math.toRadians(180));
    private Pose spike2LeverPos = new Pose(19.195, 65.495, Math.toRadians(180));
    private Pose shootFarPos = new Pose(59.701, 19.570, Math.toRadians(180));
    private Pose spike3Pos = new Pose(10.477, 35.804, Math.toRadians(180));
    private Pose preHumanPos = new Pose(9.701+3, 16.888+4, Math.toRadians(200));
    private Pose humanPos = new Pose(9.841+3, 9.009+1, Math.toRadians(200));
    private Pose parkPos = new Pose(36.05607476635514, 18.85981308411213, Math.toRadians(90));
    private Pose startToPreSpike2ControlPos = new Pose(68.561, 72.835, Math.toRadians(180));
    private Pose preSpike2ToLeverControlPos = new Pose(27.140, 59.785, Math.toRadians(180));
    private Pose leverToShootFarControlPos = new Pose(47.126, 51.089, Math.toRadians(180));
    private Pose shootFarToSpike3ControlPos = new Pose(52.136, 42.360, Math.toRadians(180));

    private PathChain startToPreSpike2Path;
    private PathChain preSpike2ToLeverPath;
    private PathChain leverToShootFarPath;
    private PathChain shootFarToSpike3Path;
    private PathChain spike3ToShootFarPath;
    private PathChain shootFarToPreHumanPath;
    private PathChain preHumanToHumanPath;
    private PathChain humanToShootFarPath;
    private PathChain shootFarToParkPath;

    private List<LynxModule> hubs;

    private TelemetryManager telemetryA;
    private IOSubsystem IO;
    private Follower follower;

    private Utils utils;
    private double alpha;
    private boolean succesDemand;
    private int id;
    private double intakeAngle = 207;

    private ElapsedTime timer;

    private int autoOutakeTimeout = 5000;

    @Override
    public void initialize() {
        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        follower = Constants.createFollower(hardwareMap);

        IO = new IOSubsystem(hardwareMap);
        register(IO);
        IO.startLimeLight();

        utils = new Utils(IO, follower);

        startPose = utils.mirror(startPose);
        preSpike2Pos = utils.mirror(preSpike2Pos);
        spike2LeverPos = utils.mirror(spike2LeverPos);
        shootFarPos = utils.mirror(shootFarPos);
        spike3Pos = utils.mirror(spike3Pos);
        preHumanPos = utils.mirror(preHumanPos);
        humanPos = utils.mirror(humanPos);
        parkPos = utils.mirror(parkPos);
        startToPreSpike2ControlPos = utils.mirror(startToPreSpike2ControlPos);
        preSpike2ToLeverControlPos = utils.mirror(preSpike2ToLeverControlPos);
        leverToShootFarControlPos = utils.mirror(leverToShootFarControlPos);
        shootFarToSpike3ControlPos = utils.mirror(shootFarToSpike3ControlPos);
        intakeAngle = 180 - intakeAngle;

        follower.setStartingPose(startPose);
        follower.setPose(startPose);
        follower.setMaxPower(1);

        telemetryA = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryA.update(telemetry);

        IO.teamIsRed = true;

        timer = new ElapsedTime();
        timer.startTime();

        startToPreSpike2Path = follower.pathBuilder()
                .addPath(new BezierCurve(
                        startPose,
                        startToPreSpike2ControlPos,
                        preSpike2Pos))
                .setConstantHeadingInterpolation(preSpike2Pos.getHeading())
                .setBrakingStrength(1)
                .setBrakingStart(1)
                .build();

        preSpike2ToLeverPath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        preSpike2Pos,
                        preSpike2ToLeverControlPos,
                        spike2LeverPos
                ))
                .setBrakingStrength(1)
                .setBrakingStart(1)
                .setConstantHeadingInterpolation(spike2LeverPos.getHeading())
                .build();

        leverToShootFarPath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        spike2LeverPos,
                        leverToShootFarControlPos,
                        shootFarPos
                ))
                .setBrakingStrength(0.7)
                .setBrakingStart(0.75)
                .setConstantHeadingInterpolation(shootFarPos.getHeading())
                .build();

        shootFarToSpike3Path = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootFarPos,
                        shootFarToSpike3ControlPos,
                        spike3Pos
                ))
                .setConstantHeadingInterpolation(spike3Pos.getHeading())
                .setBrakingStrength(1)
                .setBrakingStart(1)
                .build();

        spike3ToShootFarPath = follower.pathBuilder()
                .addPath(new BezierLine(spike3Pos, shootFarPos))
                .setConstantHeadingInterpolation(shootFarPos.getHeading())
                .setBrakingStrength(1)
                .setBrakingStart(1)
                .build();

        shootFarToPreHumanPath = follower.pathBuilder()
                .addPath(new BezierLine(shootFarPos, preHumanPos))
                .setLinearHeadingInterpolation(shootFarPos.getHeading(), Math.toRadians(intakeAngle))
                .setBrakingStrength(1)
                .setBrakingStart(1)
                .build();

        preHumanToHumanPath = follower.pathBuilder()
                .addPath(new BezierLine(preHumanPos, humanPos))
                .setConstantHeadingInterpolation(Math.toRadians(intakeAngle))
                .setBrakingStrength(1)
                .setBrakingStart(1)
                .build();

        humanToShootFarPath = follower.pathBuilder()
                .addPath(new BezierLine(humanPos, shootFarPos))
                .setLinearHeadingInterpolation(Math.toRadians(intakeAngle), shootFarPos.getHeading())
                .setBrakingStrength(1)
                .setBrakingStart(1)
                .build();

        shootFarToParkPath = follower.pathBuilder()
                .addPath(new BezierLine(shootFarPos, parkPos))
                .setLinearHeadingInterpolation(shootFarPos.getHeading(), parkPos.getHeading())
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
                        utils.newAutoOutake(5000),
                        new FollowPathCommand(follower, startToPreSpike2Path)
                                .beforeStarting(() -> follower.setMaxPower(0.55))
                                .alongWith(utils.newStartIntake(4500))
                                .andThen(utils.newStopIntake()),
                        new FollowPathCommand(follower, preSpike2ToLeverPath)
                                .beforeStarting(() -> follower.setMaxPower(1)),
                        new WaitCommand(500),
                        new FollowPathCommand(follower, leverToShootFarPath)
                                .beforeStarting(() -> follower.setMaxPower(1)),
                        utils.newAutoOutake(4500),
                        new FollowPathCommand(follower, shootFarToSpike3Path)
                                .alongWith(utils.newStartIntake(5000))
                                .beforeStarting(() -> follower.setMaxPower(0.45))
                                .andThen(utils.newStopIntake()),
                        new FollowPathCommand(follower, spike3ToShootFarPath)
                                .beforeStarting(() -> follower.setMaxPower(1)),
                        utils.newAutoOutake(3000),
                        new FollowPathCommand(follower, shootFarToPreHumanPath)
                                .beforeStarting(() -> follower.setMaxPower(0.7)),

                        new FollowPathCommand(follower, preHumanToHumanPath)
                                .beforeStarting(() -> follower.setMaxPower(0.45 ))
                                .alongWith(utils.newStartIntake(3000))
                                .andThen(utils.newStopIntake()),
                        new FollowPathCommand(follower, humanToShootFarPath)
                                .beforeStarting(() -> follower.setMaxPower(0.9)),
                        new WaitCommand(100),
                        utils.newAutoOutake(3000),
                        new InstantCommand(()->IO.setTargetTurretRads(0)),
                        new FollowPathCommand(follower, shootFarToParkPath)
                                .beforeStarting(() -> follower.setMaxPower(1)),
                        new InstantCommand(() -> {SharedUtils.sharedPose = follower.getPose();})
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
        telemetryA.update(telemetry);

        IO.update_sep_pid(timer.seconds());
        IO.update_RPM_pid();
        IO.update_turret_pid();
    }

}
