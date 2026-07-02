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
import com.pedropathing.paths.Path;
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
// 13.585046728971963, 62.587850467289724, 145
@Configurable
@Autonomous(name = "AutonomieTEST")
public class AutoTestPosesFarBackBlue extends CommandOpMode {
    public static boolean hold = false;
    public static double target_X = 31.327102803738317;
    public static double target_Y = 134.5046728971963;
    public static double target_HEADING = 270;


    // Pedro visualizer poses
    private Pose startPose = new Pose(31.327102803738317, 134.5046728971963, Math.toRadians(270));

    private Pose pastPose = new Pose(31.327102803738317, 134.5046728971963, Math.toRadians(270));
    private Pose targetPose = new Pose(31.327102803738317, 134.5046728971963, Math.toRadians(270));

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
        utils = new Utils(IO, follower);

        telemetryA = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryA.update(telemetry);

        IO.teamIsRed = false;

        timer = new ElapsedTime();
        timer.startTime();



    }

    @Override
    public void run() {
        hubs.forEach(LynxModule::clearBulkCache);
        super.run();
        follower.update();

        targetPose = new Pose(target_X,target_Y,Math.toRadians(target_HEADING));
        checkUpdate();

        telemetryA.addData("x",follower.getPose().getX());
        telemetryA.addData("y",follower.getPose().getY());
        telemetryA.addData("heading",follower.getPose().getHeading());

        telemetryA.update(telemetry);

    }


    public void checkUpdate(){
        if ((pastPose.getX()!=targetPose.getX() || pastPose.getY()!=targetPose.getY() || pastPose.getHeading()!=targetPose.getHeading())&&!hold){

            PathChain goPath = follower.pathBuilder()
                    .addPath(new BezierLine(
                            startPose,
                            targetPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(),targetPose.getHeading())
                    .setBrakingStrength(1)
                    .setBrakingStart(1)
                    .build();
            schedule(new FollowPathCommand(follower,goPath));
        }
        pastPose = targetPose;
    }

    public void updateTargetPose(){
    }

}




