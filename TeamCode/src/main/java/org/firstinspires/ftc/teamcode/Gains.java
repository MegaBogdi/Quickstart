package org.firstinspires.ftc.teamcode;


import com.bylazar.configurables.annotations.Configurable;


public class Gains {
    @Configurable
    public static class ProfileGains {
        public static double kP = 0;
        public static double kD = 0;

        public static double kJ = 0;
        public static double velCmdEps;
    }
    @Configurable
    public static class PIDFGains{
        public static double kP = 0;
        public static double kD = 0;

    }

    @Configurable
    public static class SorterGains{
        // Gains SORTER
        public static double sP =0.0006;  //0.000405
        public static double sI =0.0;   //0.00455
        public static double sD =0.000038;  //0.0000230.000085
        public static double sS = 0.0436 ;
        public static double sT = 0.068;
        public static double SORT_POS_EPS =30;
        public static double SORT_VEL_EPS =300;
        public static double pwrMin = 0.45; // above this we are trying HARD
        public static double jamTime=0.5; // above this its stuck for to long

        // Trapezoid profile (sorter ticks space, ticks/s velocity, ticks/s^2 acceleration)
        public static boolean SORT_USE_TRAPEZOID = true;
        public static double SORT_PROFILE_MAX_V = 34482;
        public static double SORT_PROFILE_MAX_A = 3104541;
        public static double SORT_PROFILE_KP = 0.00055;
        public static double SORT_PROFILE_KD = 0.00002;
        public static double SORT_PROFILE_KA = 0.0;
        public static double SORT_PROFILE_MIN_DIST = 30;
        public static double SORT_PROFILE_VEL_CMD_EPS = 80;
    }

    @Configurable
    public static class TurretGains{

        public static int target = 0;
        public static double tP=0.00022;
        public static double tI=0;
        public static double tD=0.000014;
        public static double tS=0.04;
        public static double tT=0.095;
        public static double TUR_POS_EPS =30;
        public static double TUR_VEL_EPS =5000;
        public static int MAX_TICKS =16000;
        public static int MIN_TICKS =-17500;

        public static  double TURRET_TOLERANCE = 0.08;

    }
    @Configurable
    public static class RPMGains{
        public static double kP =0.007;
        public static double kI =0.0;
        public static double kD =0.0;  // try to keep 0
        public static double kS = 0.151;   //Static power;
        public static  double kV=0.000185;  // pure feedForward
        public static double RPM_EPS = 30; // RPM TOLERANCE
    }


}
