package org.firstinspires.ftc.teamcode.Subsystems;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.Gains.RPMGains.*;
import static org.firstinspires.ftc.teamcode.Gains.SorterGains.*;
import static org.firstinspires.ftc.teamcode.Gains.TurretGains.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;


@Configurable
public class IOSubsystem extends SubsystemBase {

    private HardwareMap hmap;
    public CachingServo park;
    private final CachingCRServo turret;
    private final CachingServo push1;
    private final CachingServo push2;
    private final CachingServo hood;
    private final Limelight3A lime;

    public final CachingDcMotorEx sorter;
    private final CachingDcMotorEx collector;
    private final CachingDcMotorEx launcher1;
    private final CachingDcMotorEx launcher2;
    private final NormalizedColorSensor snsr1;
    private final CachingServo led;


    public boolean teamIsRed=false;
    public static double testRPM=1500;
    public static double testAngle=0.3;
    public static double redX=130.5;
    public static double redY=125;
    public static double testPower;
    private int space = 2730;//2730;
    private final double[] BLUE_GOAL = {10.5,134};
    private final double[] RED_GOAL = {135.5,134};

    private final double tick_per_rotation = 8192;

    public static double RPM=0;
    public static double targetRPM=0;
    public static int targetPos;
    public static int currentTurret;
    public double targetTurret = Math.toRadians(0);
    // Mechanical zero offset between robot heading and turret encoder zero.
    public static double turretHeadingOffsetRad = Math.PI;
    private long lastUpdate = 1;
    public int[] ALL = new int[3];
    public int[] MOTIF = new int[3];
    public int demand_index;
    public boolean nowOpen = false;
    public boolean demanding = false; // temportal latch for demanding
    public boolean climbing = false; // temportal latch for climb
    public double   SERVO_MIN_LIMIT = 0;
    public static double SERVO_MAX_LIMIT = 0.35;

    public double PUSH_MIN_LIMIT = 0.056;
    public double PUSH_MAX_LIMIT = 0.156;

    private double jamStart;
    public boolean jam;
    public boolean recovering;

    private PIDController pid;
    public PIDController pidT;
    private PIDController pidRPM;

    // Sorter trapezoid profile state.
    private boolean sorterProfileNeedsInit = true;
    private boolean sorterProfileActive = false;
    private double sorterProfileStartTime = 0.0;
    private double sorterProfileStartPos = 0.0;
    private double sorterProfileDistance = 0.0;
    private double sorterProfileDirection = 1.0;
    private double sorterProfileT1 = 0.0;
    private double sorterProfileT2 = 0.0;
    private double sorterProfileT3 = 0.0;
    private double sorterProfilePeakV = 0.0;
    private double sorterProfileAccelDist = 0.0;






    public IOSubsystem(final HardwareMap hMap) {
        snsr1 = hMap.get(NormalizedColorSensor.class, "senA");
        snsr1.setGain(8.0f);

        ALL[0] = 0;        MOTIF[0] = 1;
        ALL[1] = 0;        MOTIF[1] = 2;
        ALL[2] = 0;        MOTIF[2] = 1;

//===========================MOTOR==================================================

        collector = new CachingDcMotorEx((hMap.get(DcMotorEx.class, "coll")));
        collector.setDirection(DcMotorSimple.Direction.REVERSE);
        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sorter = new CachingDcMotorEx((hMap.get(DcMotorEx.class, "sort")));
        sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sorter.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher1 = new CachingDcMotorEx((hMap.get(DcMotorEx.class, "lauA"))); // ENCODER RPM
        //launcher1.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher2 = new CachingDcMotorEx((hMap.get(DcMotorEx.class, "lauB"))); // ENCODER TURRET
        //launcher2.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //==========================LED===================================
        led = new CachingServo(hMap.get(Servo.class,"led"));
        led.setPosition(0);
        //==========================SERVO=========================================

        push1 = new CachingServo(hMap.get(Servo.class,"pushA"));
        push2 = new CachingServo(hMap.get(Servo.class,"pushB"));
        push2.setDirection(Servo.Direction.REVERSE);
        push1.setPosition(PUSH_MIN_LIMIT);
        push2.setPosition(PUSH_MIN_LIMIT);

        hood =  new CachingServo(hMap.get(Servo.class,"hood"));
        hood.setDirection(Servo.Direction.REVERSE);
        hood.setPosition(SERVO_MIN_LIMIT);


        park = new CachingServo(hMap.get(Servo.class,"park"));
        park.setDirection(Servo.Direction.REVERSE);

        //=====================CRSERVO ======================================
        turret = new CachingCRServo(hMap.get(CRServo.class, "turet"));
        //turret.setPower(0);
        //turret.setDirection(DcMotorSimple.Direction.REVERSE);


        lime = hMap.get(Limelight3A.class, "limelight");
        lime.pipelineSwitch(0);
        //lime.start();



        //follower = Constants.createFollower(hMap);
        //follower.setStartingPose(new Pose(89.49532710280373, 9.196261682242984, Math.toRadians(270)));
        //follower.startTeleopDrive();
        //follower.update();

        pid = new PIDController(sP, sI, sD);
        targetPos = 0;

        pidT = new PIDController(tP,tI,tD);
        targetRPM=0;

        pidRPM = new PIDController(kP,kI,kD);
        targetRPM=0;



        //GoBildaPinpointDriver pinpoint;



        this.hmap = hMap;

    }

    public void setALIANCE(boolean ALIANCE){
        teamIsRed = ALIANCE;
    }

    public void setLed(double color){
        led.setPosition(color);
    }

/*
    double[][] shootingData = {
            { 100, 2300, 0.8 },
            {  120,   2300,  0.6},
            { 150,   2460,  0.1 },  //2460
            { 164,   2570,  0.1 }, // 2570
            { 192,   2700,  0.1 },
            { 200,   2730,  0.1 },  //
            { 250,   2960,  0.1 },
            { 264,   3000,  0.1 },
            { 350,   3460,  0.1 },
    };*/

    double[][] shootingData = {
            {108,2150,0},
            {200,2550,0.2},
            {231, 2600, 0.2},
            {312,3000,0.23},
            {343,3150,0.27}
    };




    public double[] getInterpolatedValues(double currentDistance)
    {
        if (currentDistance == -1) return new double[] {0, 0};

        if (currentDistance <= shootingData[0][0]) {
            return new double[]{ shootingData[0][1], shootingData[0][2] };
        }

        if (currentDistance >= shootingData[shootingData.length - 1][0]) {
            int last = shootingData.length - 1;
            return new double[]{ shootingData[last][1], shootingData[last][2] };
        }


        for (int i = 0; i < shootingData.length - 1; i++) {
            double distLow = shootingData[i][0];
            double distHigh = shootingData[i+1][0];

            if (currentDistance > distLow && currentDistance < distHigh) {

                double t = (currentDistance - distLow) / (distHigh - distLow);

                double rpm = shootingData[i][1] + (shootingData[i+1][1] - shootingData[i][1]) * t;
                double angle = shootingData[i][2] + (shootingData[i+1][2] - shootingData[i][2]) * t;

                return new double[]{ rpm, angle };
            }
        }
        return new double[]{0, 0};
    }

    public int getTurretTicks(){
        return launcher1.getCurrentPosition();
    }
    public double getTurretVelocity(){
        return launcher1.getVelocity();
    }


    public int getMotif(){
        LLResult result = lime.getLatestResult();

        if (result == null || !result.isValid()) return -1;
        else {
            return result.getFiducialResults().stream()
                    .map(LLResultTypes.FiducialResult::getFiducialId)
                    .filter(id -> id == 21 || id == 22 || id == 23)
                    .findFirst()
                    .orElse(-1);

        }
    }
    public void startLimeLight(){
        lime.start();
    }
    public void pauseLimeLight(){
        lime.pause();
    }
    public void stopLimeLight(){
        lime.stop();
    }


    public void motifTranslate(int id){
        if (id==-1){MOTIF[0]=2;MOTIF[1]=1;MOTIF[2]=1;}
        else if (id==21){MOTIF[0]=2;MOTIF[1]=1;MOTIF[2]=1;}
        else if (id==22){MOTIF[0]=1;MOTIF[1]=2;MOTIF[2]=1;}
        else if (id==23){MOTIF[0]=1;MOTIF[1]=1;MOTIF[2]=2;}
    }

    public boolean checkMotifFind(int id){
        if (id==-1){return false;}
        else{return true;}
    }



    public double getDistanceOdom(Pose curPose){
        double x = curPose.getX();
        double y = curPose.getY();

        double dist;
        if (teamIsRed){
            double Ry = RED_GOAL[1] - y;
            double Rx = RED_GOAL[0] - x;
            dist = Math.sqrt(Math.pow(Rx,2) + Math.pow(Ry,2));
        } else {
            double Ry = BLUE_GOAL[1] - y;
            double Rx = BLUE_GOAL[0] - x;
            dist = Math.sqrt(Math.pow(Rx,2) + Math.pow(Ry,2));
        }
        return dist*2.54;
    }

// RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM RPM


    public void setMotorRPM(double rpm){
        targetRPM = rpm;
    }
    public double returnRPM(){
        return RPM;
    }
    public double returnTargetRPM(){
        return targetRPM;
    }
    public double returnTargetPos(){
        return targetPos;
    }
    public double returnSorterPos(){
        return sorter.getCurrentPosition();
    }
    public void setLauncherPower(double power){
        launcher1.setPower(power);
        launcher2.setPower(power);
    }

// ANGLE FLYWHEEL ANGLE FLYWHEEL ANGLE FLYWHEEL ANGLE FLYWHEEL ANGLE FLYWHEEL ANGLE FLYWHEEL ANGLE FLYWHEEL ANGLE FLYWHEEL ANGLE FLYWHEEL ANGLE FLYWHEEL ANGLE FLYWHEEL
    public void setNormalizedAngle(double rawAngle) {

        rawAngle = clamp(rawAngle, SERVO_MIN_LIMIT, SERVO_MAX_LIMIT);

        hood.setPosition((rawAngle - SERVO_MIN_LIMIT) / (SERVO_MAX_LIMIT - SERVO_MIN_LIMIT));
    }

    public void setTargetTurretRads(double newTargetPos){
        targetTurret = newTargetPos;
    }


    public double getAngle(Pose pose){
        double[] goal = teamIsRed ? RED_GOAL : BLUE_GOAL;
        double dx = goal[0] - pose.getX();
        double dy = goal[1] - pose.getY();
        return Math.atan2(dy, dx);
    }

    public double turretCommandFromGoalAngle(double goalAngle, Pose pose) {
        return AngleUnit.normalizeRadians(goalAngle - (pose.getHeading() - Math.toRadians(180)));
    }


    public void setHood(double ang)
    {
        hood.setPosition(ang);

    }

// Angle Turret Angle Turret Angle Turret Angle Turret Angle Turret Angle Turret Angle Turret Angle Turret Angle Turret Angle Turret Angle Turret Angle Turret Angle Turret Angle Turret


    public double[] returnTuret(){

        return new double[] {launcher1.getCurrentPosition(), launcher1.getVelocity()};

    }
    public double returnTargetTuret(String mode){
        if ("rads".equals(mode)) { return targetTurret; }
        return rads2ticks(targetTurret);
    }

    public boolean isTurretReady(double alpha,Follower recivedFollower){
        double turretFieldHeading =  AngleUnit.normalizeRadians(-recivedFollower.getPose().getHeading() + tick2rads(getTurretTicks()) + Math.toRadians(180));
        return Math.abs(-turretFieldHeading - alpha) < TURRET_TOLERANCE;
    }



    public double testTurretReady(double alpha,Follower recivedFollower) {
        double turretFieldHeading =  AngleUnit.normalizeRadians(-recivedFollower.getPose().getHeading() + tick2rads(getTurretTicks()) + Math.toRadians(180));
        return Math.abs(-turretFieldHeading - alpha);
    }
    double ratio = 207.0/44.0;
    public int rads2ticks(double rads){
        return (int)Math.round((rads/(Math.PI*2)) * 8192 * ratio);
    }
    public double tick2rads(int tick){
        return tick * (Math.PI * 2) / ratio / 8192.0;
    }

    public int detectDominant(NormalizedRGBA c) {
        // Grab raw normalized values (0..1)
        float r = c.red;
        float g = c.green;
        float b = c.blue;

        // Optional: throw away very dim readings
        float sum = r + g + b;
        if (sum < 0.01f) return 0; // too dark / no object

        // Normalize so brightness doesn’t matter
        r /= sum;
        g /= sum;
        b /= sum;

        // Dominance thresholds
        if (r > g * 1.3f && r > b * 1.3f) {
            return 0;
        } else if (g > r * 2.0f && g > b * 1.2 && g>0.007) {
            return 2;
        } else if (b > r * 1.01f && b > g * 1.01f) {
            return 1;
        } else {
            return 0; // nothing clearly dominant
        }
    }

    public float[] getRGB() {
        NormalizedRGBA snsr = snsr1.getNormalizedColors();
        // Grab raw normalized values (0..1)
        float r = snsr.red;
        float g = snsr.green;
        float b = snsr.blue;

        return new float[] {r,g,b};

    }

    public boolean isOcupied(){
        if (detectDominant(snsr1.getNormalizedColors())==0){
            return false;}
        else{
            return true;}
    }
    public int readSensor() {
        int rez = detectDominant(snsr1.getNormalizedColors());
        return rez;
    }
    public void climb(){
        shiftSorterTarget(space);
        cycle_up();
    }
    public void climbDown(){
        shiftSorterTarget(-space);
        cycle_up();cycle_up();
    }

    public void start_intake(){collector.setPower(1);}
    public void stop_intake(){collector.setPower(0);}
    public void expel_intake(){collector.setPower(-1);}
    double TICKS_PER_REVOLUTION = 28.0;
    public double TPS2RPM(double tps){
        RPM = (tps/TICKS_PER_REVOLUTION)*60;
        return RPM;
    }

    public boolean isRPMready(){
        if (Math.abs(targetRPM-RPM)<RPM_EPS){return true;}
        else {return false;}
    }

    public void setPush(double pos){
        push1.setPosition(pos);
        push2.setPosition(pos);
    }
    public double getPush(){
        return push1.getPosition();}

    public int getTicksSort(){
        return sorter.getCurrentPosition();
    }

    public int ocupied(){
        int count = 0;
        for(int ball:ALL){
            if (ball != 0){
            count += 1;}
        }
        return count;
    }

    public void cycle_up(){
        int temp = ALL[2];
        ALL[2] = ALL[1];
        ALL[1] = ALL[0];
        ALL[0] = temp;
    }

    public void regist(){
        ALL[0] = readSensor();
    }
    public void regist_release(){
        ALL[1] = 0;
        demand_index += 1;
    }

    public void close(){
        if (nowOpen){shiftSorterTarget(space/2);}
        nowOpen = false;
    }
    public void open(){
        if (!nowOpen){shiftSorterTarget(-space/2);}
        nowOpen = true;
    }
    public void climb1inDir(){
        int dir = 1;
        double currentPos = sorter.getCurrentPosition();
        double error = Math.abs(targetPos-currentPos);
        double try1error = Math.abs((targetPos + space)-currentPos);
        if (try1error>error){dir = -dir;}
        if (dir>0){climb();}else{climbDown();}
    }
    public boolean isSorterReady(){
        if (Math.abs(sorter.getCurrentPosition()-targetPos)< SORT_POS_EPS +40){
            return true;
        } else {return false;}

    }
    public void rectify(){
        shiftSorterTarget(space/2);
    }



    public boolean getDemanded(){
        int demand = MOTIF[demand_index%3];
        if (ALL[1]==demand){
            return true;
        } else if (ALL[2]==demand){
            shiftSorterTarget(-space);
            cycle_up();
            cycle_up();
            return true;
        } else if (ALL[0]==demand){
            shiftSorterTarget(space);
            cycle_up();
            return true;
        } else {return false;}
    }






    public void update_sep_pid(double t){
        double currentPos = sorter.getCurrentPosition();
        double currentVel = sorter.getVelocity();
        double cmdPos = targetPos;
        double cmdVel = 0.0;
        double cmdAcc = 0.0;

        if (SORT_USE_TRAPEZOID) {
            if (sorterProfileNeedsInit) {
                startSorterProfile(t, currentPos, targetPos);
            }

            if (sorterProfileActive) {
                double localTime = Math.max(0.0, t - sorterProfileStartTime);
                if (localTime >= sorterProfileT3) {
                    sorterProfileActive = false;
                    cmdPos = targetPos;
                } else {
                    cmdPos = sorterProfileStartPos + sorterProfileDirection * profilePosition(localTime);
                    cmdVel = sorterProfileDirection * profileVelocity(localTime);
                    cmdAcc = sorterProfileDirection * profileAcceleration(localTime);
                }
            }
        }

        double posError  = cmdPos - currentPos;
        double finalError = targetPos - currentPos;

        double output;
        if (SORT_USE_TRAPEZOID) {
            output = SORT_PROFILE_KP * posError + SORT_PROFILE_KD * (cmdVel - currentVel) + SORT_PROFILE_KA * cmdAcc;
        } else {
            output = pid.calculate(currentPos, targetPos);
        }

        if (SORT_USE_TRAPEZOID) {
            if (Math.abs(cmdVel) > SORT_PROFILE_VEL_CMD_EPS) {
                if (Math.abs(currentVel) < SORT_VEL_EPS) {
                    output += sS * Math.signum(cmdVel);
                } else {
                    output += sT * Math.signum(cmdVel);
                }
            }
        } else {
            if (Math.abs(currentVel) < SORT_VEL_EPS){ // if static
                output+= sS *Math.signum(finalError);
            } else if (Math.abs(finalError) > SORT_POS_EPS){ // if should be moving
                output+= sT*Math.signum(finalError);
            }
        }

        if (Math.abs(finalError) < SORT_POS_EPS && Math.abs(cmdVel) < SORT_PROFILE_VEL_CMD_EPS) {
            output = 0;
        }
        output = Math.max(-1,Math.min(output,1));      //CLAMP POWER

        jam = jamDetect(t,finalError,currentVel,output);


        sorter.setPower(output);
    }

    private void shiftSorterTarget(int deltaTicks) {
        targetPos += deltaTicks;
        sorterProfileNeedsInit = true;
    }

    private void startSorterProfile(double nowSec, double currentPos, double target) {
        sorterProfileNeedsInit = false;

        double distance = Math.abs(target - currentPos);
        if (distance < SORT_PROFILE_MIN_DIST || SORT_PROFILE_MAX_V <= 0 || SORT_PROFILE_MAX_A <= 0) {
            sorterProfileActive = false;
            return;
        }

        sorterProfileActive = true;
        sorterProfileStartTime = nowSec;
        sorterProfileStartPos = currentPos;
        sorterProfileDirection = Math.signum(target - currentPos);
        sorterProfileDistance = distance;

        double accelTime = SORT_PROFILE_MAX_V / SORT_PROFILE_MAX_A;
        double accelDist = 0.5 * SORT_PROFILE_MAX_A * accelTime * accelTime;

        if (2.0 * accelDist >= sorterProfileDistance) {
            // Triangle profile.
            sorterProfileT1 = Math.sqrt(sorterProfileDistance / SORT_PROFILE_MAX_A);
            sorterProfileT2 = sorterProfileT1;
            sorterProfileT3 = 2.0 * sorterProfileT1;
            sorterProfilePeakV = SORT_PROFILE_MAX_A * sorterProfileT1;
            sorterProfileAccelDist = 0.5 * SORT_PROFILE_MAX_A * sorterProfileT1 * sorterProfileT1;
        } else {
            // Trapezoid profile.
            sorterProfileT1 = accelTime;
            sorterProfilePeakV = SORT_PROFILE_MAX_V;
            sorterProfileAccelDist = accelDist;
            double cruiseDist = sorterProfileDistance - 2.0 * sorterProfileAccelDist;
            sorterProfileT2 = sorterProfileT1 + cruiseDist / sorterProfilePeakV;
            sorterProfileT3 = sorterProfileT2 + sorterProfileT1;
        }
    }

    private double profileAcceleration(double localTime) {
        if (localTime < sorterProfileT1) return SORT_PROFILE_MAX_A;
        if (localTime < sorterProfileT2) return 0.0;
        if (localTime < sorterProfileT3) return -SORT_PROFILE_MAX_A;
        return 0.0;
    }

    private double profileVelocity(double localTime) {
        if (localTime < sorterProfileT1) return SORT_PROFILE_MAX_A * localTime;
        if (localTime < sorterProfileT2) return sorterProfilePeakV;
        if (localTime < sorterProfileT3) return sorterProfilePeakV - SORT_PROFILE_MAX_A * (localTime - sorterProfileT2);
        return 0.0;
    }

    private double profilePosition(double localTime) {
        if (localTime < sorterProfileT1) return 0.5 * SORT_PROFILE_MAX_A * localTime * localTime;
        if (localTime < sorterProfileT2) return sorterProfileAccelDist + sorterProfilePeakV * (localTime - sorterProfileT1);
        if (localTime < sorterProfileT3) {
            double rem = sorterProfileT3 - localTime;
            return sorterProfileDistance - 0.5 * SORT_PROFILE_MAX_A * rem * rem;
        }
        return sorterProfileDistance;
    }


    public boolean jamDetect(double t ,double error, double vel,double output){
        boolean shouldBeMoving = Math.abs(error)> SORT_POS_EPS+40; // if comanded to move
        boolean notMoving = Math.abs(vel)<SORT_VEL_EPS; // if velocity is stuck (not moving)
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




    public void update_turret_pid(){
        currentTurret =  getTurretTicks();
        double targetTurretTicks = Math.max(MIN_TICKS,Math.min(rads2ticks(-targetTurret), MAX_TICKS)); // clamp before it gets out of range
        double currentTurretVel = getTurretVelocity();
        double error = targetTurretTicks-currentTurret;

        double output = pidT.calculate(currentTurret,targetTurretTicks);

        if (Math.abs(error) < TUR_POS_EPS){
            output=0;
        }

        if (currentTurret > MAX_TICKS){if(output>0){output=0;}} //targetTurret= tick2rads(MIN_TICKS);} //CLAMP POWER IF IT GETS OUT OF BOUNDS but let it only exit
        if (currentTurret < MIN_TICKS){if(output<0){output=0;}} //targetTurret = tick2rads(MAX_TICKS);}

        output = Math.max(-1,Math.min(output,1));                   //CLAMP POWER
        turret.setPower(output);
    }
    public void testTS(){
        turret.setPower(tS);
    }


    public void update_RPM_pid(){
        RPM = TPS2RPM(launcher2.getVelocity());
        double error = targetRPM - RPM;

        double output = kS*Math.signum(targetRPM) + kV*targetRPM;

        output += pidRPM.calculate(RPM,targetRPM);

        if (targetRPM==0){output=0;}
        output = Math.max(-1,Math.min(output,1));


        setLauncherPower(output);
    }

    public void testThePower(double power){
        setLauncherPower(power);
    }


// 207 gear mare
    //90 gear servo
    //44 gear encoder


}


