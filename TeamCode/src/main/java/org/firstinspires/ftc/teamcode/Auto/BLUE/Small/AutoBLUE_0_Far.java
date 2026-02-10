//package org.firstinspires.ftc.teamcode.Auto.BLUE.Small;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
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
//import com.pedropathing.follower.Follower;
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
//@Autonomous(name = "AutonomieBLUE_0_far", group = "Auto Blue")
//public class AutoBLUE_0_Far extends CommandOpMode {
//    private Pose startPose = new Pose(53.98130841121496,8.897196261682247,toRad180(90));
//    private Pose parkPos = new Pose(35.81308411214954,36.34579439252337,toRad180(90));
//
//    private Pose parkSafePos = new Pose(37.60747663551402,11.77570093457945,toRad180(90));
//
//    private PathChain StartToParkPath;
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
//        IO.startLimeLight();
//        IO.teamIsRed = false;
//
//
//        telemetryA = PanelsTelemetry.INSTANCE.getTelemetry();
//        telemetryA.update(telemetry);
//
//
//        StartToParkPath = follower.pathBuilder()
//                .addPath(new BezierLine(startPose,parkSafePos))
//                .setLinearHeadingInterpolation(startPose.getHeading(),parkSafePos.getHeading())
//                .build();
//
//
//
//        schedule(
//                new SequentialCommandGroup(
//                        new WaitUntilCommand(this::opModeIsActive),
//                        new WaitCommand(20000),
//                        new FollowPathCommand(follower, StartToParkPath).setHoldEnd(true).beforeStarting(()->follower.setMaxPower(1))
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
//                        new InstantCommand(() -> IO.demand_index += 1),
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
//                                        new SequentialCommandGroup(new InstantCommand(()->{telemetryA.addLine("detecting...");telemetryA.update();}),new WaitCommand(20)),   // do nothing
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
//                        new ParallelDeadlineGroup(
//                                new WaitUntilCommand(()->IO.checkMotifFind(id)),
//                                new RunCommand(()-> id =IO.getMotif())
//                        ).withTimeout(timeout),
//                        new InstantCommand(()->{IO.motifTranslate(id);IO.stopLimeLight();})
//                );
//
//    }
//
//
//
//
//}
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
