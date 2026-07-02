package org.firstinspires.ftc.teamcode.Manual.Tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IOSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Configurable
@TeleOp(name = "ResetEncoder",group = "Tests")
public class TeleopDrive extends CommandOpMode {

    private List<LynxModule> hubs;
    private IOSubsystem IO;

    @Override
    public void initialize() {
        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        IO = new IOSubsystem(hardwareMap);
        IO.resetEncoder2();
        IO.resetEncoder();
        IO.setCoada(IO.COADA_MAX_LIMIT);
        telemetry.addLine("done reset...");
        telemetry.update();

    }

    @Override
    public void run() {
        IO.setCoada(IO.COADA_MIN_LIMIT);
        hubs.forEach(LynxModule::clearBulkCache);
        super.run();

    }

}
