package org.firstinspires.ftc.teamcode.Manual.Tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Gains;
import org.firstinspires.ftc.teamcode.Subsystems.IOSubsystem;

import java.util.List;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Configurable
@TeleOp(name = "PIDTurret",group = "Tests")
public class PIDTurret extends CommandOpMode {

    private List<LynxModule> hubs;
    private GamepadEx driver1;
    private CachingServo push1;
    private CachingServo push2;
    private CachingServo hood; // max 0.375

    private CachingCRServo turret;

    private CachingServo park;
    private Servo led;
    private double color;

    public static int Target = 0;

    private IOSubsystem IO;


    @Override
    public void initialize() {
        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        IO = new IOSubsystem(hardwareMap);

    }





    @Override
    public void run() {
        hubs.forEach(LynxModule::clearBulkCache);
        super.run();

        IO.setTargetTurretRads(Gains.TurretGains.target);
        IO.pidT.setPID(Gains.TurretGains.tP, Gains.TurretGains.tI, Gains.TurretGains.tD);


        telemetry.addData("TicksTarget",IO.getTurretTicks());
        telemetry.addData("Ticks2Rads",IO.tick2rads(IO.getTurretTicks()));
        telemetry.addData("Rads2Ticks",IO.rads2ticks(IO.targetTurret));
        telemetry.addData("Transform",IO.rads2ticks(IO.tick2rads(IO.getTurretTicks())));
        telemetry.addData("Degrees",Math.toDegrees(IO.tick2rads(IO.getTurretTicks())));


        IO.update_turret_pid();

        telemetry.update();

    }

    public void setPush(double pos){
        push1.setPosition(pos);
        push2.setPosition(pos);
    }
}
