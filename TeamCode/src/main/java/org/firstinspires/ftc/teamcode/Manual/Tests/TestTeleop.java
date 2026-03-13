package org.firstinspires.ftc.teamcode.Manual.Tests;

import static org.firstinspires.ftc.teamcode.Auto.SharedUtils.targetPos;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.IOSubsystem;

import java.util.List;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Configurable
@TeleOp(name = "TestTeleop",group = "Tests")
public class TestTeleop extends CommandOpMode {

    private List<LynxModule> hubs;
    private GamepadEx driver1;
    private CachingCRServo turret1;
    private CachingCRServo turret2;
    private CachingServo hood; // max 0.375
    private CachingServo coada; // max 0.375




    private CachingServo park1;
    private CachingServo park2;
    private Servo led;
    private double color;

    private IOSubsystem IO;


    public static double TEST = 0.1711;
    public static double TEST2 = 0.0052;

    @Override
    public void initialize() {
        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        IO = new IOSubsystem(hardwareMap);


        driver1 = new GamepadEx(gamepad1);


        turret1 = new CachingCRServo(hardwareMap.get(CRServo.class,"turA"));
        turret2 = new CachingCRServo(hardwareMap.get(CRServo.class,"turB"));
        //push2.setDirection(Servo.Direction.REVERSE);
        //setTurret(0);

        hood = new CachingServo(hardwareMap.get(Servo.class,"hood"));
        //hood.setDirection(Servo.Direction.REVERSE);
        //hood.setPosition(0);


        coada = new CachingServo(hardwareMap.get(Servo.class,"coada"));

        //turret = new CachingCRServo(hardwareMap.get(CRServo.class,"turet"));

        led = hardwareMap.get(Servo.class,"led");

//        driver1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                .whenPressed(()->hood.setPosition(0));
//        driver1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//                .whenPressed(()->hood.setPosition(0.35));
        driver1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                        .whenPressed(()->IO.setHood(IO.hood.getPosition()+0.05));
        driver1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                        .whenPressed(()->IO.setHood(IO.hood.getPosition()-0.05));
        driver1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
        driver1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);


        driver1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(()->coada.setPosition(0.05));
        driver1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(()->coada.setPosition(0.27));
        driver1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(()->coada.setPosition(coada.getPosition()+0.05));
        driver1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(()->coada.setPosition(coada.getPosition()-0.05));






    }





    @Override
    public void run() {
        hubs.forEach(LynxModule::clearBulkCache);
        super.run();




//
        telemetry.addData("TargetPos",IO.sorter.getCurrentPosition());
//        telemetry.addData("Ticks2Rads",IO.tick2rads(IO.getTurretTicks()));
//        telemetry.addData("Rads2Ticks",IO.rads2ticks(IO.targetTurret));
//        telemetry.addData("Transform",IO.rads2ticks(IO.tick2rads(IO.getTurretTicks())));
//        telemetry.addData("Degrees",Math.toDegrees(IO.tick2rads(IO.getTurretTicks())));
        telemetry.addData("hood",hood.getPosition());
//        telemetry.addData("tur:",turret1.getPosition());
        telemetry.addData("coada:",coada.getPosition());
        telemetry.addData("RPM",IO.getRPM());
        telemetry.addData("targetRPM",IO.targetRPM);



        //IO.update_turret_pid();
        IO.update_RPM_pid();
        telemetry.update();

    }


    public void setPark(double pos){
        park1.setPosition(pos);
        park2.setPosition(pos);
    }
}
