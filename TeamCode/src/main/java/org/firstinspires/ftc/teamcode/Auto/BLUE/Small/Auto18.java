package org.firstinspires.ftc.teamcode.Auto.BLUE.Small;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.Utils;
import org.firstinspires.ftc.teamcode.Subsystems.IOSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

// 13.585046728971963, 62.587850467289724, 145
@Configurable
@Autonomous(name = "Autonomie 18")
public class Auto18 extends CommandOpMode {



    // Pedro visualizer poses
    private Pose startPose = new Pose(31.327102803738317, 134.5046728971963, Math.toRadians(270));
    private Pose tempPose = new Pose(30, 60, Math.toRadians(145));
    private Pose leverPose = new Pose(12.585046728971963+0.2, 62.587850467289724+0.2, Math.toRadians(145));
    private Pose retryPose = new Pose(12.585046728971963+0.2+2, 62.587850467289724+0.2, Math.toRadians(145));
    private Pose preLeverPose = new Pose(35.3271028037, 71.5046728972, Math.toRadians(180));
    private Pose shootPose = new Pose(55.3271028037, 85.5046728972, Math.toRadians(230));
    private Pose spike1Pose = new Pose(18.5271028037, 85.5046728972-1, Math.toRadians(180));
    private Pose preSpike2Pose = new Pose(45.3271028037, 64.5046728972, Math.toRadians(230));
    private Pose spike2Pose = new Pose(12.3271028037, 60.5046728972, Math.toRadians(180));
    private Pose exitPose = new Pose(55.3271028037-15, 85.5046728972-10, Math.toRadians(230));


    private List<LynxModule> hubs;

    private TelemetryManager telemetryA;
    private IOSubsystem IO;
    private Follower follower;

    private Utils utils;

    private ElapsedTime timer;


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


        PathChain startToShootPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        startPose,
                        shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),preSpike2Pose.getHeading())
                .setBrakingStrength(1)
                .setBrakingStart(1)
                .build();


        PathChain shootToLeverPath= follower.pathBuilder()
                .addPath(new BezierLine(
                        shootPose,
                        preLeverPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(),preLeverPose.getHeading())
                .addPath(new BezierLine(
                        preLeverPose,
                        leverPose
                ))
                .setLinearHeadingInterpolation(preLeverPose.getHeading(),leverPose.getHeading())
                .setBrakingStrength(1)
                .setBrakingStart(1)
                .build();

        PathChain retryLever= follower.pathBuilder()
                .addPath(new BezierLine(
                        leverPose,
                        retryPose))
                .setConstantHeadingInterpolation(leverPose.getHeading())
                .addPath(new BezierLine(
                        retryPose,
                        leverPose
                ))
                .setConstantHeadingInterpolation(leverPose.getHeading())
                .setBrakingStrength(1)
                .setBrakingStart(1)
                .build();



        PathChain leverToShootPath= follower.pathBuilder()
                .addPath(new BezierLine(
                        leverPose,
                        preLeverPose
                ))
                .setLinearHeadingInterpolation(leverPose.getHeading(),shootPose.getHeading())
                .addPath(new BezierLine(
                        preLeverPose,
                        shootPose))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .setBrakingStrength(1)
                .setBrakingStart(1)
                .build();

        PathChain shootTOspike1Path = follower.pathBuilder()
                .addPath(new BezierLine(
                        shootPose,
                        spike1Pose
                ))
                .setConstantHeadingInterpolation(spike1Pose.getHeading())
                .setBrakingStrength(1)
                .setBrakingStart(1)
                .build();

        PathChain spike1TOshootPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        spike1Pose,
                        shootPose
                ))
                .setLinearHeadingInterpolation(spike1Pose.getHeading(),shootPose.getHeading())
                .setBrakingStrength(1)
                .setBrakingStart(1)
                .build();

        PathChain shootTOspike2 =  follower.pathBuilder()
                .addPath(new BezierLine(
                        shootPose,
                        preSpike2Pose
                ))
                .setLinearHeadingInterpolation(preSpike2Pose.getHeading(),spike2Pose.getHeading())
                .addPath(new BezierLine(
                        preSpike2Pose,
                        spike2Pose
                ))
                .setConstantHeadingInterpolation(spike2Pose.getHeading())
                .setBrakingStrength(1)
                .setBrakingStart(1)
                .build();
        PathChain spike2TOshoot =  follower.pathBuilder()
                .addPath(new BezierLine(
                        spike2Pose,
                        preLeverPose
                ))
                .setLinearHeadingInterpolation(spike2Pose.getHeading(),preLeverPose.getHeading())
                .addPath(new BezierLine(
                        preLeverPose,
                        shootPose
                ))
                .setLinearHeadingInterpolation(preLeverPose.getHeading(),shootPose.getHeading())
                .setBrakingStrength(1)
                .setBrakingStart(1)
                .build();

        PathChain exit = follower.pathBuilder()
                .addPath(new BezierLine(
                        shootPose,
                        exitPose
                ))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .setBrakingStrength(1)
                .setBrakingStart(1)
                .build();





        schedule(new SequentialCommandGroup(
                new WaitUntilCommand(this::opModeIsActive),
                new InstantCommand(() -> {
                    IO.ALL[0] = 0;
                    IO.ALL[1] = 0;
                    IO.ALL[2] = 0;
                    IO.setMotorRPM(2600);
                    IO.targetTurret=Math.toRadians(90);
                }),
                new FollowPathCommand(follower, startToShootPath),
//==========================PRELOAD SEQUENCE ========================
                 utils.newAutoOutake(4000,1),

//======================== SPIKE 2 SEQUENCE ============================
                new FollowPathCommand(follower, shootTOspike2)
                        .alongWith(utils.newStartIntake(4000)),
                new FollowPathCommand(follower, spike2TOshoot),
                utils.newAutoOutake(4000,1),


//======================== SPIKE 1 SEQUENCE ==============================
                new FollowPathCommand(follower,shootTOspike1Path)
                        .alongWith(utils.newStartIntake(3000)),
                new FollowPathCommand(follower,spike1TOshootPath),
                utils.newAutoOutake(4000,1),




//============================= EXIT =====================================
                new FollowPathCommand(follower,exit)





        ));
    }

    @Override
    public void run() {
        hubs.forEach(LynxModule::clearBulkCache);
        super.run();
        follower.update();


        telemetryA.addData("x",follower.getPose().getX());
        telemetryA.addData("y",follower.getPose().getY());
        telemetryA.addData("heading",follower.getPose().getHeading());
        telemetryA.addData("time dif",timer.milliseconds()- IO.last_recived);
        telemetryA.addData("lastRecived",IO.last_recived);
        telemetryA.addData("time",timer.milliseconds());

        telemetryA.update(telemetry);

        IO.update_sep_pid(timer.seconds());
        IO.update_RPM_pid();
        IO.update_turret_pid();

    }



}




