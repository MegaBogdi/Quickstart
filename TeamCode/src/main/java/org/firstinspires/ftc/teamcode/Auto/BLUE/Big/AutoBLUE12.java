//package org.firstinspires.ftc.teamcode.Auto.BLUE.Big;
//
//import com.arcrobotics.ftclib.command.Command;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.ConditionalCommand;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
//import com.arcrobotics.ftclib.command.RepeatCommand;
//import com.arcrobotics.ftclib.command.RunCommand;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.arcrobotics.ftclib.command.WaitUntilCommand;
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.Subsystems.IOSubsystem;
//import org.firstinspires.ftc.teamcode.Subsystems.commands.FollowPathCommand;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//@Configurable
//@Autonomous(name = "AutonomieBLUE_12", group = "Auto Blue")
//public class AutoBLUE12 extends CommandOpMode {
//    private Pose startPose = new Pose(24.224,126.953,toRad180(136));
//    private Pose shootPos = new Pose(58.542,84.336,toRad180(180));
//    private Pose shootFarPos =  new Pose(60.5607476635514,11.214953271028039,toRad180(90));
//    private Pose colectSpike1Pos = new Pose(16.373831775700936,83.66355140186916,toRad180(180));
//    private Pose colectSpike2Pos = new Pose(11,60.33644859813084,toRad180(180));
//    private Pose helpSpike2Pos = new Pose(62.35514018691589,56.299065420560744,toRad180(180));
//    private Pose colectSpike3Pos = new Pose(11,36.560747663551396,toRad180(180));
//    private Pose helpSpike3Pos = new Pose(73.1214953271028,29.831775700934557,toRad180(180));
//    private Pose pullPos = new Pose(16.373831775700936,69.98130841121497 ,toRad180(180));
//    private Pose helpPullPos = new Pose(32.299065420560744,62.3551401869159 ,toRad180(180));
//    private Pose helpPullReturnPos = new Pose(48.44859813084112,70.20560747663552 ,toRad180(180));
//
//    private Pose parkPos = new Pose(35.66355140186916,33.86915887850466,toRad180(90));
//
//    private PathChain StartToShootPath, ShootToSpike2Path, Spike2ToLeverPath, LeverToShootPath, ShootToSpike1Path,Spike1ToShootPath,ShootToSpike3Path, Spike3ToShootFarPath,ShootFarToParkPath;
//    private TelemetryManager telemetryA;
//    private IOSubsystem IO;
//    private Follower follower;
//    private double alpha;
//    private boolean succesDemand;
//    private int id;
//
//    @Override
//    public void initialize(){
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startPose);
//        follower.setPose(startPose);
//        follower.setMaxPower(0.8);
//        IO = new IOSubsystem(hardwareMap);
//        register(IO);
//        IO.teamIsRed = false;
//
//
//        telemetryA = PanelsTelemetry.INSTANCE.getTelemetry();
//        telemetryA.update(telemetry);
//
//
//
//
//
//
//        StartToShootPath = follower.pathBuilder()
//                .addPath(new BezierLine(startPose,shootPos))
//                .setLinearHeadingInterpolation(startPose.getHeading(),shootPos.getHeading())
//                .build();
//        ShootToSpike2Path = follower.pathBuilder()
//                .addPath(new BezierCurve(shootPos,helpSpike2Pos,colectSpike2Pos))
//                .setConstantHeadingInterpolation(shootPos.getHeading())
//                .build();
//        Spike2ToLeverPath = follower.pathBuilder()
//                .addPath(new BezierCurve(colectSpike2Pos,helpPullPos,pullPos))
//                .setConstantHeadingInterpolation(shootPos.getHeading())
//                .build();
//        LeverToShootPath = follower.pathBuilder()
//                .addPath(new BezierCurve(pullPos,helpPullReturnPos,shootPos))
//                .setConstantHeadingInterpolation(shootPos.getHeading())
//                .build();
//        ShootToSpike1Path = follower.pathBuilder()
//                .addPath(new BezierLine(shootPos,colectSpike1Pos))
//                .setConstantHeadingInterpolation(shootPos.getHeading())
//                .build();
//        Spike1ToShootPath = follower.pathBuilder()
//                .addPath(new BezierLine(colectSpike1Pos,shootPos))
//                .setConstantHeadingInterpolation(shootPos.getHeading())
//                .build();
//        ShootToSpike3Path= follower.pathBuilder()
//                .addPath(new BezierCurve(shootPos,helpSpike3Pos,colectSpike3Pos))
//                .setConstantHeadingInterpolation(shootPos.getHeading())
//                .build();
//        Spike3ToShootFarPath = follower.pathBuilder()
//                .addPath(new BezierLine(colectSpike3Pos,shootFarPos))
//                .setConstantHeadingInterpolation(shootPos.getHeading())
//                .build();
//        ShootFarToParkPath = follower.pathBuilder()
//                .addPath(new BezierLine(shootFarPos,parkPos))
//                .setConstantHeadingInterpolation(parkPos.getHeading())
//                .build();
//
//
//        schedule(
//                new SequentialCommandGroup(
//                        new WaitUntilCommand(this::opModeIsActive),
//                        new FollowPathCommand(follower, StartToShootPath).beforeStarting(()->follower.setMaxPower(1)),
//                        newStartIntake(4000).andThen(newStopIntake()).alongWith(new SequentialCommandGroup(new InstantCommand(()->IO.setTargetTurretRads(-1.5)),newMotify(3000))),
//                        newAutoOutake(4500),
//                        new FollowPathCommand(follower, ShootToSpike2Path).alongWith(newStartIntake(5000)).beforeStarting(()->follower.setMaxPower(0.6)),
//
//                        new FollowPathCommand(follower, Spike2ToLeverPath).beforeStarting(()->follower.setMaxPower(0.4)),
//                        new WaitCommand(250),
//
//                        new FollowPathCommand(follower, LeverToShootPath).beforeStarting(()->follower.setMaxPower(1)),
//                        newAutoOutake(4500),
//
//                        new FollowPathCommand(follower, ShootToSpike1Path).beforeStarting(()->follower.setMaxPower(0.7)).alongWith(newStartIntake(5000)).andThen(newStopIntake()),
//
//                        new FollowPathCommand(follower, Spike1ToShootPath).beforeStarting(()->follower.setMaxPower(1)),
//                        newAutoOutake(4500),
//                        new FollowPathCommand(follower, ShootToSpike3Path).beforeStarting(()->follower.setMaxPower(0.7)).alongWith(newStartIntake(10000)).andThen(newStopIntake()),
//
//                        new FollowPathCommand(follower, Spike3ToShootFarPath).beforeStarting(()->follower.setMaxPower(1)),
//                        newAutoOutake(10000),
//                        new FollowPathCommand(follower, ShootFarToParkPath).beforeStarting(()->follower.setMaxPower(1)),
//                        new WaitCommand(2000)
//
//                )
//        );
//
//
//    }
//
//    @Override
//    public void run() {
//        super.run();              // runs CommandScheduler
//        follower.update();        // keep follower alive
//
//        Pose p = follower.getPose();
//        telemetryA.debug("dist: " + IO.getDistanceOdom(follower.getPose()));
//        telemetryA.debug(String.format("coord: x=%.2f  y=%.2f  h=%.1f", p.getX(), p.getY(), Math.toDegrees(p.getHeading())));
//        //telemetry.addData("sug coaie:",1);
//        telemetryA.debug(String.format("ALL: 0=%d | 1=%d | 2=%d", IO.ALL[0], IO.ALL[1], IO.ALL[2]));
//        telemetryA.debug(String.format("condition: RPM:%b | turret:%b,sorter:%b", IO.isRPMready(), IO.isTurretReady(alpha, follower), IO.isSorterReady()));
//        telemetryA.debug("rpm: " + IO.returnRPM());
//        telemetryA.debug("targetRpm: " + IO.returnTargetRPM());
//        telemetryA.debug("motify: " + IO.getMotif());
//        telemetryA.update(telemetry);
//
//        IO.update_sep_pid();
//        IO.update_RPM_pid();
//        IO.update_turret_pid();
//    }
//
//    public double toRad180(double degrees){
//        return AngleUnit.normalizeRadians(Math.toRadians(degrees)+Math.toRadians(180));
//    }
//
//    private Command newAutoOutake(int timeout){
//        Command quickPush = new SequentialCommandGroup(
//                new InstantCommand(() -> IO.setPush(0.3)),
//                new WaitCommand(200),
//                new InstantCommand(() -> IO.setPush(0)),
//                new WaitCommand(200),
//                new InstantCommand(() -> IO.regist_release())
//        );
//        Command Demand = new SequentialCommandGroup(
//                new InstantCommand(() -> succesDemand = IO.getDemanded()),
//                new ConditionalCommand(
//                        new SequentialCommandGroup(
//                                new WaitUntilCommand(() -> (IO.isSorterReady() && IO.isRPMready() && IO.isTurretReady(alpha, follower))),
//                                quickPush
//                        ),
//                        new InstantCommand(() -> {IO.demand_index += 1;telemetryA.debug("SEARCHING>>>>>>>>");}),
//                        () -> succesDemand
//                )
//        );
//        Command interp = new InstantCommand(() -> {
//            double dist = IO.getDistanceOdom(follower.getPose());
//            double[] interpValues = IO.getInterpolatedValues(dist);
//            IO.setHood(interpValues[1]);
//            IO.setMotorRPM(interpValues[0]);
//        });
//        Command autoAim = new InstantCommand(() -> {
//            alpha = IO.getAngle(follower.getPose());
//            double curAlpha = AngleUnit.normalizeRadians(follower.getPose().getHeading() + Math.toRadians(180)); //facem asta pentru ca fata robotului real e defapt spatele robotului virtual
//            IO.setTargetTurretRads(-(AngleUnit.normalizeRadians(curAlpha - alpha)));
//        });
//
//        Command stopAutoOutake = new InstantCommand(() -> {
//            IO.setMotorRPM(0);
//            IO.setHood(0.1);
//            IO.setTargetTurretRads(0);
//            IO.setPush(0);
//        });                   // complete?
//        return new SequentialCommandGroup(
//                new ParallelDeadlineGroup(
//                        new WaitUntilCommand(() -> IO.ocupied() == 0),  //shoot until empty
//                        new RepeatCommand(interp),
//                        new RepeatCommand(autoAim),
//                        new RepeatCommand(Demand)
//                ).withTimeout(timeout),
//                stopAutoOutake
//        );
//
//    }
//
//    private Command newStartIntake(int timeout){
//        Command stopIntake = new InstantCommand(() -> {
//            IO.stop_intake();
//            IO.close();
//            IO.setPush(0);
//        });
//
//        return
//                new SequentialCommandGroup(
//                        new InstantCommand(()->IO.open()),
//                        new ParallelDeadlineGroup(
//                                new WaitUntilCommand(()->IO.ocupied() >= 3),  // while not full
//                                new RunCommand(()->IO.start_intake()),
//                                new RepeatCommand(new ConditionalCommand(
//                                        new SequentialCommandGroup(
//                                                new WaitCommand(50),
//                                                new InstantCommand(() -> IO.regist()),// register the incoming ball in memory
//                                                new InstantCommand(() -> IO.climb()), // phisicly spin the sorter
//                                                new WaitUntilCommand(() -> !IO.isOcupied())
//                                        ),
//                                        new SequentialCommandGroup(new InstantCommand(()->{telemetryA.debug("detecting...");telemetryA.update(telemetry);}),new WaitCommand(20)),   // do nothing
//                                        () -> (IO.isOcupied()) // if recived ball
//                                ))
//
//                        ).withTimeout(timeout),
//                        stopIntake
//                );
//
//    }
//
//    private Command newStopIntake(){
//        return new InstantCommand(() -> {
//            IO.stop_intake();
//            IO.close();
//            IO.setPush(0);
//        });
//    }
//
//    private Command newMotify(int timeout){
//        return
//                new SequentialCommandGroup(
//                    new ParallelDeadlineGroup(
//                        new WaitUntilCommand(()->IO.checkMotifFind(id)),
//                        new RunCommand(()-> id =IO.getMotif())
//                    ).withTimeout(timeout),
//                new InstantCommand(()->IO.motifTranslate(id))
//        );
//
//    }
//
//
//
//
//    }
//
///* uai deci:
//
//* nu e bun demand intexu pentru ca isi ia offset cand nu are ce ii trebuie
//-fa un demand_index_offset
//
//
//
//
//*/
