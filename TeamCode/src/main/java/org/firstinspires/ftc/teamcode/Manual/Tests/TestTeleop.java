package org.firstinspires.ftc.teamcode.Manual.Tests;

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
    private CachingServo push1;
    private CachingServo push2;
    private CachingServo hood; // max 0.375

    private CachingCRServo turret;

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


        push1 = new CachingServo(hardwareMap.get(Servo.class,"pushA"));
        push2 = new CachingServo(hardwareMap.get(Servo.class,"pushB"));
        //push2.setDirection(Servo.Direction.REVERSE);
        setPush(0.056);

        hood = new CachingServo(hardwareMap.get(Servo.class,"hood"));
        //hood.setDirection(Servo.Direction.REVERSE);
        //hood.setPosition(0);

        park1 = new CachingServo(hardwareMap.get(Servo.class,"parkA"));
        //park1.setDirection(Servo.Direction.REVERSE);
        park2 = new CachingServo(hardwareMap.get(Servo.class,"parkB"));
        //park2.setDirection(Servo.Direction.REVERSE);

        turret = new CachingCRServo(hardwareMap.get(CRServo.class,"turet"));

        led = hardwareMap.get(Servo.class,"led");

        driver1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(()->hood.setPosition(0));
        driver1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(()->hood.setPosition(0.35));
        driver1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(()->setPush(push1.getPosition()-0.001));
        driver1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(()->setPush(push1.getPosition()+0.001));

        double offset = 0.026;
        driver1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(()->setPark(0.5));
        driver1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(()->setPark(0.1));
        driver1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(()->setPark(1));
        driver1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(()->setPark(0));






    }





    @Override
    public void run() {
        hubs.forEach(LynxModule::clearBulkCache);
        super.run();

        //IO.targetTurret += IO.tick2rads((int)driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 2);
        //IO.targetTurret -= IO.tick2rads((int) (driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * 2));


//
//        telemetry.addData("TicksTarget",IO.getTurretTicks());
//        telemetry.addData("Ticks2Rads",IO.tick2rads(IO.getTurretTicks()));
//        telemetry.addData("Rads2Ticks",IO.rads2ticks(IO.targetTurret));
//        telemetry.addData("Transform",IO.rads2ticks(IO.tick2rads(IO.getTurretTicks())));
//        telemetry.addData("Degrees",Math.toDegrees(IO.tick2rads(IO.getTurretTicks())));
        telemetry.addData("hood",hood.getPosition());
        telemetry.addData("push:",push1.getPosition());

        //IO.update_turret_pid();
        telemetry.update();

    }

    public void setPush(double pos){
        push1.setPosition(pos);
        push2.setPosition(pos);
    }
    public void setPark(double pos){
        park1.setPosition(pos);
        park2.setPosition(pos);
    }
}
