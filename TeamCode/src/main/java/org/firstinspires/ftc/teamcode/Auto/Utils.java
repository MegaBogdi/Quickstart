package org.firstinspires.ftc.teamcode.Auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Subsystems.IOSubsystem;

public class Utils {
    Follower follower;
    IOSubsystem IO;
    public Utils(IOSubsystem IO, Follower follower){
        this.IO = IO;
        this.follower = follower;
    }
    boolean succesDemand;
    double alpha;
    int id;

    public Command newAutoOutake(int timeout){
        Command quickPush = new SequentialCommandGroup(
                new InstantCommand(() -> IO.setPush(IO.PUSH_MAX_LIMIT)),
                new WaitCommand(35),
                new InstantCommand(() -> IO.setPush(IO.PUSH_MIN_LIMIT)),
                new WaitCommand(35),
                new InstantCommand(() -> IO.regist_release())
        );
        Command Demand = new SequentialCommandGroup(
                new InstantCommand(() -> succesDemand = IO.getDemanded()),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> (IO.isSorterReady() && IO.isRPMready())),
                                quickPush
                        ),
                        new InstantCommand(() -> IO.demand_index += 1),
                        () -> succesDemand
                )
        );



        Command interp = new InstantCommand(() -> {
            double dist = IO.getDistanceOdom(follower.getPose());
            double[] interpValues = IO.getInterpolatedValues(dist);
            IO.setHood(interpValues[1]);
            IO.setMotorRPM(interpValues[0]);
        });
        Command autoAim = new InstantCommand(() -> {
            alpha = IO.getAngle(follower.getPose());
            IO.setTargetTurretRads(IO.turretCommandFromGoalAngle(alpha, follower.getPose()));
        });

        Command autoOutake = new ParallelCommandGroup(
                autoAim,
                interp,
                new ConditionalCommand(
                        Demand,
                        new InstantCommand(() -> {}),
                        () -> IO.isTurretReady(alpha, follower)
                )
        );

        Command stopAutoOutake = new InstantCommand(() -> {
            IO.setMotorRPM(1750);
            IO.setHood(0.1);
            IO.setTargetTurretRads(0);
            IO.setPush(IO.PUSH_MIN_LIMIT);
        });                   // complete?
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(() -> IO.ocupied() == 0),  //shoot until empty
                        new RepeatCommand(autoOutake)
                ).withTimeout(timeout),
                stopAutoOutake
        );

    }


    public Command newStartIntake(int timeout){
        Command stopIntake = new InstantCommand(() -> {
            IO.stop_intake();
            IO.close();
            IO.setPush(0);
        });

        return
                new SequentialCommandGroup(
                        new InstantCommand(()->IO.open()),
                        new ParallelDeadlineGroup(
                                new WaitUntilCommand(()->IO.ocupied() >= 3),  // while not full
                                new RunCommand(()->IO.start_intake()),
                                new RepeatCommand(new ConditionalCommand( // if ball incoming
                                        new SequentialCommandGroup(
                                                new WaitCommand(50),
                                                new InstantCommand(() -> IO.regist()),// register the incoming ball in memory
                                                new InstantCommand(() -> IO.climb()), // phisicly spin the sorter
                                                new WaitUntilCommand(() -> !IO.isOcupied())
                                        ),
                                        new WaitCommand(20),   // do nothing
                                        () -> (IO.isOcupied()) // if recived ball
                                ))

                        ).withTimeout(timeout),
                        stopIntake
                );

    }


    public Command newMotify(int timeout){
        return
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                new WaitUntilCommand(()->IO.checkMotifFind(id)),
                                new RunCommand(()-> id =IO.getMotif())
                        ).withTimeout(timeout),
                        new InstantCommand(()->{IO.motifTranslate(id);IO.stopLimeLight();})
                );

    }

    public Command newStopIntake(){
        return new InstantCommand(() -> {
            IO.stop_intake();
            IO.close();
            IO.setPush(0);
        });
    }



    public Command createRecoverySeq() {
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

    public Command newAutoAim(){
        return new InstantCommand(() -> {
            alpha = IO.getAngle(follower.getPose());
            IO.setTargetTurretRads(IO.turretCommandFromGoalAngle(alpha, follower.getPose()));
        });
    }




    public Pose mirror(Pose pose){
        return new Pose( 145-pose.getX(), 145-pose.getY(), Math.toRadians(180)-pose.getHeading());
    }


}
