package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Gains.RPMGains.kL;

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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Manual.LimePoseSync;
import org.firstinspires.ftc.teamcode.Subsystems.IOSubsystem;

public class Utils {
    Follower follower;
    IOSubsystem IO;
    double[] interpValues;
    public Utils(IOSubsystem IO, Follower follower){
        this.IO = IO;
        this.follower = follower;
    }
    boolean succesDemand;
    double alpha;
    public int id;
    public int motif=0;



    public Command newAutoOutake(int timeout,int spike){
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(new WaitUntilCommand(()->IO.isRPMready()),new RunCommand(()->interp()),new RunCommand(()->autoAim()),new InstantCommand(()->{IO.expel_intake();})),
                new ParallelDeadlineGroup(new WaitUntilCommand(()->IO.isSorterReady()),new InstantCommand(()->{IO.stop_intake();IO.shiftSorterTarget(IO.space*(motifData[spike-1][motif]));})),
                new InstantCommand(()->{IO.stop_intake();IO.two_spin();IO.pastPos=IO.sorter.getCurrentPosition();}),
                new WaitUntilCommand(()->IO.isOneRevPast()),
                new InstantCommand(()->IO.setCoada(IO.COADA_MAX_LIMIT)),
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(()->IO.isSorterReady()),
                        new RunCommand(()-> ladderInterp()),
                        new RunCommand(()->autoAim())
                ).withTimeout(timeout),
                new InstantCommand(()->{IO.setCoada(IO.COADA_MIN_LIMIT);}),   //LimePoseSync.sync(follower, IO.lime, -IO.tick2rads(IO.getTurretPos()));
                new InstantCommand(()->{IO.ALL[0]=0;IO.ALL[1]=0;IO.ALL[2]=0;}),
                new InstantCommand(()->stopAutoOutake())
        );

    }

    public void interp(){

        interpValues = IO.getInterpolatedValues(IO.getDistanceOdom(follower.getPose()));
        IO.setHood(interpValues[1]);
        IO.setMotorRPM(interpValues[0]);
    }
    public void ladderInterp(){
        double error = IO.targetRPM - IO.getRPM();
        interpValues = IO.getInterpolatedValues(IO.getDistanceOdom(follower.getPose()));
        IO.setMotorRPM(interpValues[0]);
        IO.setHood(interpValues[1]-error*kL);
    }
    public void autoAim(){
        alpha = IO.getAngle(follower.getPose());
        IO.setTurretPosRads(IO.turretCommandFromGoalAngle(alpha, follower.getPose()));
    }
    int [][] motifData= {
            { 0,-1, 1},
            { 1, 0,-1},
            {-1, 1, 0}
    };
    public void stopAutoOutake(){
        IO.setMotorRPM(2300);
        IO.setHood(IO.SERVO_MIN_LIMIT);
        IO.setTurretPosRads(IO.teamIsRed? Math.toRadians(-90):Math.toRadians(90));
        IO.setCoada(IO.COADA_MIN_LIMIT);
    }





    public Command newStartIntake(int timeout){
        Command stopIntake = new InstantCommand(() -> {
            IO.stop_intake();
            IO.close();
            IO.setPush(0);
        });

        return
                new SequentialCommandGroup(
                        new InstantCommand(()->{IO.open();IO.set_intake(0.9);}),
                        new ParallelDeadlineGroup(
                                new WaitUntilCommand(()->IO.ocupied() >= 3),  // while not full
                                new RepeatCommand(new ConditionalCommand( // if ball incoming
                                        new SequentialCommandGroup(
                                                new WaitCommand(15),
                                                new InstantCommand(() -> IO.regist()),// register the incoming ball in memory
                                                new InstantCommand(() -> IO.climb()), // phisicly spin the sorter
                                                new WaitUntilCommand(() -> !IO.isOcupied())
                                        ),
                                        new InstantCommand(),   // do nothing
                                        () -> (IO.isOcupied()) // if recived ball
                                ))

                        ).withTimeout(timeout),
                        stopIntake
                );

    }

    public Command newStopIntake(){
        return new InstantCommand(() -> {
            IO.stop_intake();
            IO.close();
            IO.setPush(0);
        });
    }


    public Command newMotify(int timeout){
        return
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                new WaitUntilCommand(()->IO.checkMotifFind(id)),
                                new InstantCommand(()->IO.setLed(0.388)),
                                new RunCommand(()-> id =IO.getMotif())
                        ).withTimeout(timeout),
                        new InstantCommand(()->{
                           if (id == -1){IO.setLed(0.277);}
                           else{IO.setLed(0.500);}
                        }),
                        new InstantCommand(()->{motif = IO.motifTranslate(id);})
                );

    }



    public Pose mirror(Pose pose){
        return new Pose(
                145 - pose.getX(),
                pose.getY(),
                AngleUnit.normalizeRadians(Math.PI - pose.getHeading())
        );
    }





}
