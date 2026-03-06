package org.firstinspires.ftc.teamcode.Manual;


import static org.firstinspires.ftc.teamcode.Auto.SharedUtils.targetPos;
import static org.firstinspires.ftc.teamcode.Gains.RPMGains.kL;
import static org.firstinspires.ftc.teamcode.Manual.Drawing.drawDebug;
import static org.firstinspires.ftc.teamcode.Manual.Drawing.drawRobot;

import org.firstinspires.ftc.teamcode.Auto.SharedUtils;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IOSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

@Configurable
@TeleOp(name = "ClassyMovement")
public class ClassyMovement extends CommandOpMode {

    private List<LynxModule> hubs;

    private IMU imu;
    private ElapsedTime timer;
    private boolean ALLIANCE = false; // Default to False (Blue)
    private IOSubsystem IO;
    private DriveSubsystem chassis;

    GamepadEx driver2;
    GamepadEx driver1;

    //IMU imu;

    private TelemetryManager telemetryA;
    private double[] interpValues;

    private boolean controlOverride = false;

    private Follower follower;
    private Pose defaultPoseBLUE = new Pose(113.19626168224296, 8.897196261682275, Math.toRadians(90));  // blue side horizontal
    private Pose defaultPoseRED = new Pose(30.97196261682239, 8.448598130841154, Math.toRadians(90));  // blue side horizontal


    private boolean succesDemand;
    private boolean motified =false;
    private double lastCameraRefreshSec = -1.0;
    private double headingOffset = Math.toRadians(90);
    double alpha;
    int id=-1;

    @Override
    public void initialize() {
        // -------------------------------------------------------------------------
        // 1. HARDWARE INITIALIZATION
        // -------------------------------------------------------------------------
        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        timer = new ElapsedTime();
        timer.startTime();



        // -------------------------------------------------------------------------
        // 2. SUBSYSTEMS & FOLLOWER
        // -------------------------------------------------------------------------
        chassis = new DriveSubsystem(hardwareMap);
        IO = new IOSubsystem(hardwareMap);

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(SharedUtils.sharedPose);

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        // -------------------------------------------------------------------------
        // 3. TELEMETRY (NEW PANELS FORMAT)
        // -------------------------------------------------------------------------
        telemetryA = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryA.update(telemetry); // push initial frame to DS + Panels

        // -------------------------------------------------------------------------
        // 4. COMMAND BINDINGS
        // -------------------------------------------------------------------------
        register(IO);

        IO.close();
        IO.startLimeLight();

        Command quickPush = new SequentialCommandGroup(
                new InstantCommand(() -> IO.setPush(IO.PUSH_MAX_LIMIT)),
                new WaitCommand(35),
                new InstantCommand(() -> IO.setPush(IO.PUSH_MIN_LIMIT)),
                new WaitCommand(35),
                new InstantCommand(() -> IO.regist_release())
        );

        Command correctify = new SequentialCommandGroup(
                new InstantCommand(() -> IO.open()),
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(() -> IO.isSorterReady()),
                        new RepeatCommand(new SequentialCommandGroup(
                                new InstantCommand(() -> IO.climb1inDir()),
                                new WaitCommand(500)
                        ))
                ),
                new ConditionalCommand(
                        new InstantCommand(),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> IO.climb1inDir()),
                                new WaitCommand(500)
                        ),
                        () -> IO.isSorterReady()
                ),
                new InstantCommand(() -> IO.ALL[0] = 0)
        );

        Command scan_ = new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(() -> IO.checkMotifFind(id)),
                        new RunCommand(() -> id = IO.getMotif()),
                        new RunCommand(()->gamepad1.rumble(200))
                ).withTimeout(3000),
                new InstantCommand(() -> {
                    IO.motifTranslate(id);
                    IO.stopLimeLight();
                })
        );

        Command scan = new ConditionalCommand(
            new ConditionalCommand(
                new InstantCommand(()->{IO.motifTranslate(id);IO.stopLimeLight();motified=true;}),
                new RunCommand(() -> {id = IO.getMotif(); gamepad1.rumble(200);telemetryA.addLine("searching");}),
                ()->IO.checkMotifFind(id)
            ),
            new InstantCommand(()->telemetryA.debug(motified)),
            ()->!motified
        );

        Command stopAutoOutake = new InstantCommand(() -> {
            if (IO.ocupied()<1) {
                IO.setMotorRPM(0);
                IO.setHood(0.1);
                IO.setTargetTurretRads(0);
            }
        });


        Command autoOutake_ = new SequentialCommandGroup(
                new ParallelDeadlineGroup(new WaitUntilCommand(()->IO.isRPMready()),new RunCommand(()->interpF()),new RunCommand(()->autoAimF())),
                new InstantCommand(()->{IO.two_spin();IO.pastPos=IO.sorter.getCurrentPosition();}),
                new WaitUntilCommand(()->IO.isOneRevPast()),
                new InstantCommand(()->IO.setCoada(IO.COADA_MAX_LIMIT)),
//                new InstantCommand(()->IO.full_spin()),
//                new WaitUntilCommand(()->IO.isSorterReady()),
//                new InstantCommand(()->{IO.full_spin();IO.setCoada(IO.COADA_MAX_LIMIT);}),
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(()->IO.isSorterReady()),
                        new RunCommand(()-> ladderInterp()),
                        new RunCommand(()->autoAimF())
                ),
                new InstantCommand(()->IO.setCoada(IO.COADA_MIN_LIMIT)),
                new InstantCommand(()->{IO.ALL[0]=0;IO.ALL[1]=0;IO.ALL[2]=0;}),
                stopAutoOutake
        );



        // --- Driver 1 Bindings ---

        driver1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .and(new Trigger(() -> !driver1.isDown(GamepadKeys.Button.BACK)));

        driver1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .and(new Trigger(() -> !driver1.isDown(GamepadKeys.Button.BACK)));

        driver1.getGamepadButton(GamepadKeys.Button.X)
                .and(new Trigger(() -> !driver1.isDown(GamepadKeys.Button.BACK)))
                .whileActiveContinuous(() -> {
                    IO.startLimeLight();
                    IO.setTargetTurretRads(0);
                    LimePoseSync.sync(follower, IO.lime, IO.getTurretTarget());
                    follower.update();
                })
                .whenInactive(() -> IO.stopLimeLight());


        driver1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .and(new Trigger(() -> !driver1.isDown(GamepadKeys.Button.BACK)))
                        .whenActive(()->IO.climb());



        driver1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .and(new Trigger(() -> !driver1.isDown(GamepadKeys.Button.BACK)))
                .whenActive(() -> IO.setCoada(IO.COADA_MAX_LIMIT));

        driver1.getGamepadButton(GamepadKeys.Button.BACK)
                .and(new Trigger(() -> driver1.isDown(GamepadKeys.Button.A)))
                .whenActive(correctify);

        driver1.getGamepadButton(GamepadKeys.Button.BACK)
                .and(new Trigger(() -> driver1.isDown(GamepadKeys.Button.B)))
                .whenActive(() -> IO.rectify());

        driver1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(() -> chassis.setPowerLimit(0.5))
                .whenReleased(() -> chassis.setPowerLimit(1.0));

        driver1.getGamepadButton(GamepadKeys.Button.START)
                .and(new Trigger(() -> driver1.isDown(GamepadKeys.Button.Y)))
                .whenActive(new InstantCommand(() -> {
                    headingOffset = follower.getHeading();
                }))
                .whenActive(() -> gamepad1.rumble(400));

        driver1.getGamepadButton(GamepadKeys.Button.START)
                .and(new Trigger(() -> driver1.isDown(GamepadKeys.Button.X)))
                .whenActive(() ->  { follower.setPose(ALLIANCE ? defaultPoseRED : defaultPoseBLUE); } )
                .whenActive(() -> gamepad1.rumble(400));

        driver1.getGamepadButton(GamepadKeys.Button.A)
                .and(new Trigger(() -> !driver1.isDown(GamepadKeys.Button.START)))
                .and(new Trigger(() -> !driver1.isDown(GamepadKeys.Button.BACK)))
                .whenActive(new InstantCommand(() -> IO.full_spin()));

        driver1.getGamepadButton(GamepadKeys.Button.Y)
                .and(new Trigger(() -> !driver1.isDown(GamepadKeys.Button.START)))
                .and(new Trigger(() -> !driver1.isDown(GamepadKeys.Button.BACK)))
                .whenActive(()->IO.setTargetTurretRads(Math.toRadians(90)));

        new Trigger(() -> driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.2)
                .whileActiveContinuous(() -> {
                    IO.expel_intake();
                    gamepad1.rumble(20);
                })
                .whenInactive(() -> IO.stop_intake());


        Command startIntake = new ConditionalCommand(
                new SequentialCommandGroup( new InstantCommand(()->IO.setLed(1)),
                new ParallelCommandGroup(
                        new InstantCommand(() -> IO.start_intake()),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new WaitCommand(50), // here for antijaming
                                        new InstantCommand(() -> IO.regist()),
                                        new InstantCommand(() -> IO.climb()),
                                        new WaitUntilCommand(() -> !IO.isOcupied())
                                ),
                                new InstantCommand(() -> {
                                }),
                                () -> IO.isOcupied()
                        )
                )),
                new InstantCommand(() -> {gamepad1.rumble(20);IO.setLed(0.5);}),
                () -> IO.ocupied() < 3
        );

        Command stopIntake = new InstantCommand(() -> {
            IO.stop_intake();
            IO.close();
            IO.setPush(0);
        });

        driver1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .and(new Trigger(() -> !IO.jam))
                .whenActive(new ConditionalCommand(
                        new InstantCommand(() -> IO.open()),
                        new InstantCommand(),
                        () -> IO.ocupied() < 3
                ))
                .whileActiveContinuous(startIntake)
                .whenInactive(stopIntake);

        driver1.getGamepadButton(GamepadKeys.Button.B)
                .and(new Trigger(()->!driver1.isDown(GamepadKeys.Button.BACK)))
                .whenActive(()->IO.togglePark());

        Command interp = new InstantCommand(() -> {
            double dist = IO.getDistanceOdom(follower.getPose());
            interpValues = IO.getInterpolatedValues(dist);
            IO.setHood(interpValues[1]);
            IO.setMotorRPM(interpValues[0]);
        });

        Command refreshPosition = new InstantCommand(() -> LimePoseSync.sync(follower, IO.lime, IO.getTurretTarget()));


        Command Demand = new SequentialCommandGroup(
                new InstantCommand(() -> succesDemand = IO.getDemanded()),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> IO.isSorterReady() && IO.isRPMready() && IO.isTurretReady(alpha,follower)),
                                quickPush
                        ),
                        new InstantCommand(() -> IO.demand_index += 1),
                        () -> succesDemand
                )
        );

        Command autoAim = new InstantCommand(() -> {
            alpha = IO.getAngle(follower.getPose());
            IO.setTargetTurretRads(IO.turretCommandFromGoalAngle(alpha, follower.getPose()));
        });


        Command autoOutake = new ParallelCommandGroup(
                new RunCommand(()->autoAimF()),
                new RunCommand(()->interpF()),
                Demand
        );






        new Trigger(() -> driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.2)
                .and(new Trigger(() -> !IO.jam))
                .whenActive(new InstantCommand(() -> IO.close()))
                .whenActive(autoOutake_);


        schedule(
                new RunCommand(() -> {
                    if (IO.jam && !IO.recovering) {
                        IO.recovering = true;
                        schedule(createRecoverySeq());
                    }
                })

        );

        // -------------------------------------------------------------------------
        // 5. ALLIANCE SELECTION LOOP
        // -------------------------------------------------------------------------
        while (opModeInInit()) {
            if (gamepad1.dpad_up) {
                ALLIANCE = true;
                IO.teamIsRed = true;
            }  // RED
            if (gamepad1.dpad_down) {
                ALLIANCE = false;
                IO.teamIsRed = false;
            } // BLUE

            telemetryA.debug("CHOOSE ALLIANCE");
            telemetryA.debug((ALLIANCE ? "> " : "  ") + "RED ALLIANCE");
            telemetryA.debug((ALLIANCE ? "  " : "> ") + "BLUE ALLIANCE");
            telemetryA.debug(IO.sorter.getCurrentPosition());
            telemetryA.update(telemetry);
        }


        // -------------------------------------------------------------------------
        // 6. APPLY CONFIGURATION
        // -------------------------------------------------------------------------

    }

    private void onOpModeEnd() {
        SharedUtils.sharedPose = follower.getPose();
    }

    private Command createRecoverySeq() {
        AtomicBoolean thenOpen = new AtomicBoolean(false);
        AtomicBoolean thenCoada = new AtomicBoolean(false);
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    thenOpen.set(IO.nowOpen);
                    IO.open();
                    IO.targetTurret = IO.closestToIntakePos()+ IO.sorter.getCurrentPosition();
                    thenCoada.set(IO.nowCoada);
                    IO.setCoada(IO.COADA_MIN_LIMIT);
                }),
                new WaitUntilCommand(()->IO.isSorterReady()),
                new ParallelDeadlineGroup(new WaitUntilCommand(IO::isOcupied),new RunCommand(()->IO.start_intake())),
                new SequentialCommandGroup(
                        new WaitCommand(50),
                        new InstantCommand(IO::regist),
                        new InstantCommand(IO::climb),
                        new WaitUntilCommand(() -> !IO.isOcupied()),
                        new InstantCommand(()-> {if(!thenOpen.get()){IO.close();}})
                ),
                new InstantCommand(() -> {
                    IO.jam = false;
                    IO.recovering = false;
                    //if(thenCoada.get()){IO.setCoada(IO.COADA_MAX_LIMIT);}else{IO.setCoada(IO.COADA_MIN_LIMIT);}
                })
        );
    }


    @Override
    public void run() {
        hubs.forEach(LynxModule::clearBulkCache);

        if (!opModeIsActive() || isStopRequested()) onOpModeEnd();

        super.run();

        follower.update();
        drawDebug(follower);

        IO.update_sep_pid(timer.seconds());
        IO.update_RPM_pid();
        IO.update_turret_pid();
        if (IO.ocupied()>=1){windUp();}

        double heading = follower.getPose().getHeading();
        telemetryA.addData("angle error", Math.toDegrees(IO.testTurretReady(IO.getAngle(follower.getPose()),follower)));
        telemetryA.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        chassis.updateSpeeds(driver1.getLeftY(), driver1.getLeftX(), driver1.getRightX(), heading - headingOffset + Math.toRadians(90));
        telemetryA.addData("x",follower.getPose().getX());
        telemetryA.addData("y",follower.getPose().getY());
        telemetryA.addData("isTurretReady",IO.isTurretReady(alpha,follower));
        telemetryA.addData("sorterPos",IO.sorter.getCurrentPosition());
        //telemetryA.addData("target Turret",IO.targetTurret);
        //telemetryA.addData("turret",IO.turret1.getPosition());
        //telemetryA.addData("proxim",IO.getProxim());
        //telemetryA.debug(String.format("ALL: 0=%d | 1=%d | 2=%d", IO.ALL[0], IO.ALL[1], IO.ALL[2]));



//        telemetryA.debug(String.format("SORTER: target=%.4f  pos=%.4f  error=%.4f",
//                IO.returnTargetPos(), IO.returnSorterPos(), (IO.returnTargetPos() + IO.returnSorterPos())));
//
//        telemetryA.debug(String.format("FLYWHEEL: target=%.2f  rpm=%.2f  error=%.2f",
//                IO.returnTargetRPM(), IO.returnRPM(), (IO.returnTargetRPM() - IO.returnRPM())));
//
//        telemetryA.debug(String.format("TURRET: target=%.2f  ticks=%.2f  error=%.2f",
//                IO.returnTargetTuret("ticks"), IO.getTurretTicks(), (IO.returnTargetTuret("ticks") - IO.returnTuret()[0])));
//
//

        telemetryA.addData("park",IO.park1.getPosition());
        telemetryA.update(telemetry);


    }

    private double wrapDeg360(double deg) {
        double out = deg % 360.0;
        if (out < 0) out += 360.0;
        return out;
    }

    private double wrapDeg180(double deg) {
        double out = (deg + 180.0) % 360.0;
        if (out < 0) out += 360.0;
        return out - 180.0;
    }



    public void windUp() {
        if (gamepad1.right_trigger < 0.2 && !driver1.isDown(GamepadKeys.Button.X)) {
            double alpha = IO.getAngle(follower.getPose());
            IO.setTargetTurretRads(IO.turretCommandFromGoalAngle(alpha, follower.getPose()));

            interpValues = IO.getInterpolatedValues(IO.getDistanceOdom(follower.getPose()));
            IO.setHood(interpValues[1]);
            IO.setMotorRPM(interpValues[0]);

        }
    }

    public void autoAimF(){
        alpha = IO.getAngle(follower.getPose());
        IO.setTargetTurretRads(IO.turretCommandFromGoalAngle(alpha, follower.getPose()));
    }
    public void interpF(){
        interpValues = IO.getInterpolatedValues(IO.getDistanceOdom(follower.getPose()));
        IO.setHood(interpValues[1]);
        IO.setMotorRPM(interpValues[0]);
    }

    public void decideBottleneck(){
        int conditions = ( IO.isSorterReady() ? 1:0 )+ (IO.isTurretReady(alpha,follower)? 1:0 )+( IO.isRPMready()?1:0);
        if (conditions==1){
            if (!IO.isSorterReady()){IO.setLed(0.611);}
            else if (!IO.isTurretReady(alpha,follower)){IO.setLed(0.722);}
            else if (!IO.isRPMready()){IO.setLed(0.33);}
        }
    }

    public void ladderInterp(){
        double error = IO.targetRPM - IO.getRPM();
        interpValues = IO.getInterpolatedValues(IO.getDistanceOdom(follower.getPose()));
        IO.setMotorRPM(interpValues[0]);
        IO.setHood(interpValues[1]-error*kL);
    }


}





