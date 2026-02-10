package org.firstinspires.ftc.teamcode.Manual.Tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TrapezoidProfile;

import java.util.List;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Configurable
@TeleOp(name = "SorterTest",group="Tests")
public class SorterTest extends CommandOpMode {

    private List<LynxModule> hubs;
    private GamepadEx driver1;
    private ElapsedTime timer;


    private CachingDcMotorEx sorter;
    private TrapezoidProfile sorter_profile;
    private int space = 2730;
    public static double SORTER_ENCODER_TPR = 8192.0;

    double vPrev,tPrev;
    double maxA,maxV;
    int ticksPrev;


    @Override
    public void initialize() {
        timer = new ElapsedTime();
        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        driver1 = new GamepadEx(gamepad1);

        sorter = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class,"sort"));
        sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sorter.setDirection(DcMotorSimple.Direction.REVERSE);


        tPrev = timer.seconds();
        ticksPrev = sorter.getCurrentPosition();
        vPrev = 0;

        driver1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(()->sorter.setPower(1));
        driver1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(()->sorter.setPower(0));
        driver1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(()->sorter.setPower(0.25));
        driver1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(()->sorter.setPower(0.5));






    }

    @Override
    public void run() {
        hubs.forEach(LynxModule::clearBulkCache);
        super.run();
        double tNow = timer.seconds();
        int ticksNow = sorter.getCurrentPosition();

        double dt = tNow - tPrev;
        // Derive velocity from encoder ticks so units are guaranteed ticks/second.
        double vNow = (ticksNow - ticksPrev) / dt;
        double aNow = (vNow - vPrev) / dt;          // ticks/sec^2
        double aAbs = Math.abs(aNow);

        maxV = Math.max(maxV, Math.abs(vNow));
        maxA = Math.max(maxA, aAbs);

        double rps = vNow / SORTER_ENCODER_TPR;
        double rpm = rps * 60.0;

        telemetry.addData("vNowTicksPerSec", vNow);
        telemetry.addData("aNowTicksPerSec2", aNow);
        telemetry.addData("maxVelTicksPerSec", maxV);
        telemetry.addData("maxAccelTicksPerSec2", maxA);
        telemetry.addData("rps", rps);
        telemetry.addData("rpm", rpm);
        telemetry.addData("apiVelRaw", sorter.getVelocity());

        vPrev = vNow;
        ticksPrev = ticksNow;
        tPrev = tNow;

        telemetry.addData("ticks", ticksNow);
        telemetry.update();

    }


    public void update_sorter_profile(double t){
        double vel = sorter.getVelocity();
        double pos = sorter.getCurrentPosition();
        double accCmd = sorter_profile.accel(t);
        double velCmd = sorter_profile.velocity(t);
        double posCmd = sorter_profile.position(t);

        double fb = sorter_profile.feedBack(velCmd,posCmd,vel,pos);
        double ff = sorter_profile.feedForward(velCmd,accCmd,vel);
        double output = fb+ff;
        output = Math.min(1,Math.max(-1,output));

        // sorter.setPower(output);
    }
}
