package org.firstinspires.ftc.teamcode.Manual;


import static org.firstinspires.ftc.teamcode.Manual.Drawing.drawDebug;
import static org.firstinspires.ftc.teamcode.Manual.Drawing.drawRobot;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Auto.SharedUtils;
import org.firstinspires.ftc.teamcode.Manual.Drawing;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IOSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.lang.annotation.Target;
import java.util.List;

@Configurable
@TeleOp(name = "ClassyMovement")
public class ClassyMovement extends CommandOpMode {

    private List<LynxModule> hubs;
    private ElapsedTime timer;
    private boolean ALLIANCE = false; // Default to False (Blue)
    private IOSubsystem IO;
    private DriveSubsystem chassis;

    GamepadEx driver2;
    GamepadEx driver1;

    IMU imu;

    private TelemetryManager telemetryA;
    private double[] interpValues;

    private boolean controlOverride = false;

    private Follower follower;
    private Pose defaultPoseBLUE = new Pose(113.19626168224296, 8.897196261682275, Math.toRadians(90));  // blue side horizontal
    private Pose defaultPoseRED = new Pose(30.97196261682239, 8.448598130841154, Math.toRadians(90));  // blue side horizontal

    private boolean succesDemand;
    private double lastCameraRefreshSec = -1.0;
    private double headingOffset = Math.toRadians(90);
    double alpha;
    int id;

    @Override
    public void initialize() {
        // -------------------------------------------------------------------------
        // 1. HARDWARE INITIALIZATION
        // -------------------------------------------------------------------------
        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        timer = new ElapsedTime();
        timer.startTime();

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();

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


        // --- Driver 1 Bindings ---

        driver1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .and(new Trigger(() -> !driver1.isDown(GamepadKeys.Button.BACK)))
                .whenActive(() -> IO.setMotorRPM(IO.returnTargetRPM() + 500));

        driver1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .and(new Trigger(() -> !driver1.isDown(GamepadKeys.Button.BACK)))
                .whenActive(() -> IO.setMotorRPM(IO.returnTargetRPM() - 500));

        driver1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .and(new Trigger(() -> !driver1.isDown(GamepadKeys.Button.BACK)))
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(() -> IO.setPush(IO.PUSH_MAX_LIMIT)),
                        new WaitCommand(35),
                        new InstantCommand(() -> IO.setPush(IO.PUSH_MIN_LIMIT)),
                        new WaitCommand(35),
                        new InstantCommand(() -> IO.regist_release())
                ));

        driver1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .and(new Trigger(() -> !driver1.isDown(GamepadKeys.Button.BACK)))
                .whenActive(() -> IO.setHood(IO.SERVO_MAX_LIMIT));

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
                .whenActive(() -> follower.setPose(ALLIANCE ? defaultPoseRED : defaultPoseBLUE))
                .whenActive(() -> gamepad1.rumble(400));

        driver1.getGamepadButton(GamepadKeys.Button.A)
                .and(new Trigger(() -> !driver1.isDown(GamepadKeys.Button.START)))
                .and(new Trigger(() -> !driver1.isDown(GamepadKeys.Button.BACK)))
                .whenActive(new InstantCommand(() -> IO.climb()));

        driver1.getGamepadButton(GamepadKeys.Button.Y)
                .and(new Trigger(() -> !driver1.isDown(GamepadKeys.Button.START)))
                .and(new Trigger(() -> !driver1.isDown(GamepadKeys.Button.BACK)))
                .whenActive(() -> IO.demand_index = 0)
                .whenActive(newMotify(1000));

        new Trigger(() -> driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.2)
                .whileActiveContinuous(() -> {
                    IO.expel_intake();
                    gamepad1.rumble(20);
                })
                .whenInactive(() -> IO.stop_intake());


        Command startIntake = new ConditionalCommand(
                new ParallelCommandGroup(
                        new InstantCommand(() -> IO.start_intake()),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        //new WaitCommand(50),
                                        new InstantCommand(() -> IO.regist()),
                                        new InstantCommand(() -> IO.climb()),
                                        new WaitUntilCommand(() -> !IO.isOcupied())
                                ),
                                new InstantCommand(() -> {
                                }),
                                () -> IO.isOcupied()
                        )
                ),
                new InstantCommand(() -> gamepad1.rumble(20)),
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

        Command interp = new InstantCommand(() -> {
            double dist = IO.getDistanceOdom(follower.getPose());
            interpValues = IO.getInterpolatedValues(dist);
            IO.setHood(interpValues[1]);
            IO.setMotorRPM(interpValues[0]);
        });

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
                autoAim,
                interp,
                Demand
        );


        Command stopAutoOutake = new InstantCommand(() -> {
            if (IO.ocupied()<1) {
                IO.setMotorRPM(0);
                IO.setHood(0.1);
                IO.setTargetTurretRads(0);
            }
        });

        new Trigger(() -> driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.2)
                .and(new Trigger(() -> !IO.jam))
                .whenActive(new InstantCommand(() -> IO.close()))
                .whileActiveContinuous(autoOutake)
                .whenInactive(stopAutoOutake);

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
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    IO.open();
                    IO.climbDown();
                }),
                new WaitUntilCommand(IO::isOcupied),
                new SequentialCommandGroup(
                        new WaitCommand(50),
                        new InstantCommand(IO::regist),
                        new InstantCommand(IO::climb),
                        new WaitUntilCommand(() -> !IO.isOcupied())
                ),
                new InstantCommand(() -> {
                    IO.jam = false;
                    IO.recovering = false;
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


    public Command newMotify(int timeout) {
        return
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                new WaitUntilCommand(() -> IO.checkMotifFind(id)),
                                new RunCommand(() -> id = IO.getMotif())
                        ).withTimeout(timeout),
                        new InstantCommand(() -> {
                            IO.motifTranslate(id);
                            IO.stopLimeLight();
                        }),
                        new InstantCommand(() -> {
                            if (IO.checkMotifFind(id)) gamepad1.rumble(300);
                        })
                );

    }

    public void windUp() {
        if (gamepad1.right_trigger < 0.2) {
            alpha = IO.getAngle(follower.getPose());
            IO.setTargetTurretRads(IO.turretCommandFromGoalAngle(alpha, follower.getPose()));
            IO.setMotorRPM(1500);
        }
    }





}





