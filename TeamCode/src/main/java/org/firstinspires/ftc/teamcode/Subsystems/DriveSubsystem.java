
package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
@Configurable
public class DriveSubsystem extends SubsystemBase {

    private PIDController pidTurn;

    public static double kP =1;
    public static double kI =0.0;
    public static double kD =0.5;


    private double powerLimit = 1.0;
    private final CachingDcMotorEx FL;
    private final CachingDcMotorEx FR;
    private final CachingDcMotorEx BL;
    private final CachingDcMotorEx BR;

    private int probe=0;

    public DriveSubsystem(final HardwareMap hardwareMap) {
        this(hardwareMap, "FL", "FR",
                "BL", "BR");
    }

    private DriveSubsystem(HardwareMap hardwareMap, String leftFront, String rightFront, String leftBack, String rightBack) {

        pidTurn = new PIDController(kP,kI,kD);
        //pidTurn.setTolerance(Math.toRadians(1));

        FL = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, leftFront));
        FR = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, rightFront));
        BL = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, leftBack));
        BR = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, rightBack));



        probe = 2;

        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public double[] updateSpeeds(double y, double x, double rx, double botHeading) {
        rx = rx*0.85 ;
        // The equivalent button is start on Xbox-style controllers.
        // Rotate the movement direction counter to the bot's rotation
        //yawScalar = 1;
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Strafe gain
        rotY = -rotY;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX - rx) / denominator;
        double frontRightPower = (rotY - rotX + rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        FL.setPower(frontLeftPower * powerLimit);
        BL.setPower(-backLeftPower * powerLimit);
        FR.setPower(-frontRightPower * powerLimit);
        BR.setPower(backRightPower * powerLimit);
        double[] ret = {frontLeftPower,backLeftPower,frontRightPower,backRightPower};
        return ret;
    }



    public double updateTurnPID(double error){
        double output = pidTurn.calculate(0,error);
        if (pidTurn.atSetPoint()){output=0;}
        output = Math.max(-1,Math.min(output,1));

        return output;
    }




    public void powerTest(double rx){
        //double rx = 1;
        FL.setPower(rx);
        BL.setPower(rx);
        FR.setPower(-rx);
        BR.setPower(-rx);
    }

    public int testProbe(){
        return probe;
    }




    public void setPowerLimit(double limit) {
        powerLimit = limit;
    }
}