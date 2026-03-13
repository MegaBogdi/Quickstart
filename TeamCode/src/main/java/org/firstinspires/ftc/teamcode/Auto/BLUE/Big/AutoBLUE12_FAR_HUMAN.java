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
@Autonomous(name = "AutonomieBLUE_12_FAR_HUMAN", group = "Auto Blue")
public class AutoBLUE12_FAR_HUMAN extends CommandOpMode {
    // Pedro visualizer poses
    private Pose startPose = new Pose(59.019, 6.857, Math.toRadians(180));
    private Pose preSpike2Pos = new Pose(50.242, 59.485-1, Math.toRadians(180));
    private Pose spike2Pos = new Pose(11.374+2.7, 59.935-1, Math.toRadians(180));
    private Pose leverPos = new Pose(19.195, 65.495, Math.toRadians(180));
    private Pose shootFarPos = new Pose(59.701, 19.570, Math.toRadians(180));
    private Pose spike3Pos = new Pose(10.477+2, 35.804, Math.toRadians(180));
    private Pose preHumanPos = new Pose(9.701+2, 16.888+4+2, Math.toRadians(200));
    private Pose humanPos = new Pose(9.841+2, 9.009+1.5, Math.toRadians(200));
    private Pose parkPos = new Pose(36.05607476635514, 18.85981308411213, Math.toRadians(90));

    private PathChain startToPreSpike2Path;
    private PathChain preSpike2ToSpike2Path;
    private PathChain spike2ToLeverPath;
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
    private double intakeAngle = 250;

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
        //IO.BLUE_GOAL[0]-=5;
        utils = new Utils(IO, follower);

        telemetryA = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryA.update(telemetry);

        IO.teamIsRed = false;

        timer = new ElapsedTime();
        timer.startTime();

//        startToPreSpike2Path = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        startPose,
//                        new Pose(72.561, 70.835),
//                        spike2Pos))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .setBrakingStrength(1)
//                .setBrakingStart(1)
//                .build();




        startToPreSpike2Path = follower.pathBuilder()
                .addPath(new BezierLine(
                        startPose,
                        preSpike2Pos))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStrength(1)
                .setBrakingStart(1)
                .build();

        preSpike2ToSpike2Path = follower.pathBuilder()
                .addPath(new BezierLine(
                        preSpike2Pos,
                        spike2Pos))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStrength(1)
                .setBrakingStart(1)
                .build();

        spike2ToLeverPath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        spike2Pos,
                        new Pose(27.140, 59.785),
                        leverPos
                ))
                .setBrakingStrength(1)
                .setBrakingStart(1)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        leverToShootFarPath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        leverPos,
                        new Pose(47.126, 51.089),
                        shootFarPos
                ))
                .setBrakingStrength(1)
                .setBrakingStart(1)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shootFarToSpike3Path = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootFarPos,
                        new Pose(52.136, 42.360),
                        spike3Pos
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStrength(1)
                .setBrakingStart(1)
                .build();

        spike3ToShootFarPath = follower.pathBuilder()
                .addPath(new BezierLine(spike3Pos, shootFarPos))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStrength(1)
                .setBrakingStart(1)
                .build();

        shootFarToPreHumanPath = follower.pathBuilder()
                .addPath(new BezierLine(shootFarPos, preHumanPos))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(intakeAngle))
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
                .setLinearHeadingInterpolation(Math.toRadians(intakeAngle), Math.toRadians(180))
                .setBrakingStrength(1)
                .setBrakingStart(1)
                .build();

        shootFarToParkPath = follower.pathBuilder()
                .addPath(new BezierLine(shootFarPos, parkPos))
                .setLinearHeadingInterpolation(Math.toRadians(180), parkPos.getHeading())
                .build();





        schedule(
                new RunCommand(() -> {
                    if (IO.jam && !IO.recovering) {
                        IO.recovering = true;
                        schedule(IO.createRecoverySeq());
                    }
                }),
                new SequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        new InstantCommand(() -> {
                            IO.ALL[0] = 1;
                            IO.ALL[1] = 2;
                            IO.ALL[2] = 1;
                            //IO.offsetGoal(false,-15,-10);
                        }),
                        new ParallelCommandGroup(
                                utils.newMotify(1000),
                                new InstantCommand(() -> IO.setTurretPosRads(Math.toRadians(90)))
                        ),
                        utils.newAutoOutake(5000,2),
                        new FollowPathCommand(follower, startToPreSpike2Path).beforeStarting(()->follower.setMaxPower(1)),
                        new FollowPathCommand(follower, preSpike2ToSpike2Path)
                                .beforeStarting(() -> follower.setMaxPower(0.41))
                                .alongWith(utils.newStartIntake(3000))
                                .andThen(utils.newStopIntake()),
                        new FollowPathCommand(follower, spike2ToLeverPath)
                                .beforeStarting(() -> follower.setMaxPower(0.9)),
                        new WaitCommand(800),
                        new FollowPathCommand(follower, leverToShootFarPath)
                                .beforeStarting(() -> follower.setMaxPower(1)),
                        new WaitCommand(400),
                        utils.newAutoOutake(4500,2),

                        new FollowPathCommand(follower, shootFarToSpike3Path)
                                .beforeStarting(() -> follower.setMaxPower(0.41))
                                .alongWith(utils.newStartIntake(5000))
                                .andThen(utils.newStopIntake()),
                        new FollowPathCommand(follower, spike3ToShootFarPath)
                                .beforeStarting(() -> follower.setMaxPower(1)),
                        new WaitCommand(300),
                        utils.newAutoOutake(3000,3),

                        new FollowPathCommand(follower, shootFarToPreHumanPath)
                                .beforeStarting(() -> follower.setMaxPower(0.7)),
                        new FollowPathCommand(follower, preHumanToHumanPath).withTimeout(4000)
                                .beforeStarting(() -> follower.setMaxPower(0.43))
                                .alongWith(utils.newStartIntake(4000))
                                .andThen(utils.newStopIntake()),
                        new FollowPathCommand(follower, humanToShootFarPath)
                                .beforeStarting(() -> follower.setMaxPower(0.9)),
                        new WaitCommand(300),
                        utils.newAutoOutake(3000,2),

                        new InstantCommand(()->IO.setTurretPosRads(0)),
                        new FollowPathCommand(follower, shootFarToParkPath)
                                .beforeStarting(() -> follower.setMaxPower(1)),
                        new InstantCommand(() -> {
                            SharedUtils.sharedPose = follower.getPose();
                            //IO.offsetGoal(false,15,10);
                        })
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




