package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Gains.PIDFGains;
import org.firstinspires.ftc.teamcode.Gains.ProfileGains;

public class TrapezoidProfile {
    boolean useProfile;
    double d,maxV, maxA;
    double velEps,velCmdEps;
    double t1,t2,t3;
    double kS, kT, kJ, kP, kD;
    double dAccel, dCruise;
    double minVelCmd, jamVel, pwrMin, jamTime;
    double jamStart = -1;
    

    public TrapezoidProfile(double d, double maxV, double maxA,boolean useProfile){
        this.useProfile = useProfile;
        this.d = d;
        if (useProfile) {
            this.maxV = maxV;
            this.maxA = maxA;
            this.velCmdEps = ProfileGains.velCmdEps;
            this.kJ = ProfileGains.kJ;
            this.kP = ProfileGains.kP;
            this.kD = ProfileGains.kD;
        } else{
            this.kP = PIDFGains.kP;
            this.kD = PIDFGains.kD;
        }








        if (useProfile) {
            t1 = maxV / maxA;
            this.dAccel = 0.5 * maxA * t1 * t1;

            if (2 * dAccel >= d) {
                //le triangle
                t1 = Math.sqrt(this.d / maxA);
                t2 = t1;
                t3 = 2 * t1;

                this.maxV = maxA * t1;
                this.dAccel = 0.5 * maxA * t1 * t1; // recopute dAccel if triang;le
                this.dCruise = 0;
            } else {
                //trapezoid
                this.dCruise = this.d - 2 * dAccel; // distance left to cruise between the accel and decel slope.
                t2 = t1 + this.dCruise / maxV;
                t3 = t2 + t1;
            }
        }
    }

    public double accel(double t){
        if(t<t1){return maxA;} // accel
        else if(t<t2){return 0;} // cruise
        else if(t<t3){return -maxA;} // decel
        return 0;
    }
    public double velocity(double t){
        if (t<t1){return maxA*t;}
        else if (t<t2){return maxV;}         // in triangle case maxV is actualy peak velocity
        else if (t<t3){return maxV-(maxA*(t-t2));}
        return 0;
    }

    public double position(double t){
        if (t<t1){return 0.5*maxA*t*t;} // integrate from accel down to dist
        else if (t<t2){return dAccel + maxV*(t-t1);}  // this one is clear (d(t1) + v*t2->3)
        else if (t<t3){return this.d -( 0.5*maxA*(t3-t)*(t3-t));} // integrate backwards as if accelerating to find the remaining distance and then subtract from whole (basicly the error instead of progress)
        return this.d;                           // delta t^2
        // in triangle case maxV is actualy peak velocity
    }

    public double feedForward(double velCmd, double accCmd, double vel) {
        double output = accCmd * kJ;
        if (Math.abs(velCmd)>velCmdEps) {  // if commanded to move
            if (Math.abs(vel) < velEps) {  // if it starts from stationary
                output += kS * Math.signum(velCmd); // static friction coef * sign of motion
            } else {
                output += kT * Math.signum(velCmd); // coulomb friction * sign of motion (direction)
            }
        }
        return output;
    }

    public double feedBack(double velCmd, double posCmd, double vel, double pos){
        double velError = velCmd-vel;
        double posError = posCmd-pos;

        double output = kP*posError + kD*velError;
        return output;
    }

    public boolean jamDetection(double t, double vel,double velCmd, double output){
        boolean shouldBeMoving = Math.abs(velCmd)>minVelCmd; // if comanded to move
        boolean notMoving = Math.abs(vel)<jamVel; // if velocity is stuck (not moving)
        boolean tryingHard = Math.abs(output) > pwrMin; // if aplying enough power
        // can add shouldBeMoving with pos additional to vel;

        if (shouldBeMoving && notMoving && tryingHard){
            if (jamStart<0){
                jamStart = t;
            }
        } else { jamStart =-1;}

        boolean jammed = jamStart>=0 && (t-jamStart)>jamTime; // if jammed and jammed enough time than its oficialy jamed;
        return jammed;
    }

    public void update_profile(double t,double vel, double pos){ //run every loop
        double accCmd,velCmd,posCmd;
        if (useProfile) {
            accCmd = accel(t);
            velCmd = velocity(t);
            posCmd = position(t);
        } else {
            accCmd = 0;
            velCmd = 0;
            posCmd = d; // Target pos
        }

        double fb = feedBack(velCmd,posCmd,vel,pos); // feed back for pos AND velocity
        double ff = feedForward(velCmd,accCmd,vel);  // feedForward
        double output = fb+ff;
        output = Math.min(1,Math.max(-1,output)); //clamp for sfety
        boolean jammed = jamDetection(t,vel,velCmd,output);

    }

}

