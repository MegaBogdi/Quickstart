// MT1 version (hard snap + orientation/offset + telemetry state)
package org.firstinspires.ftc.teamcode.Manual;

import static org.firstinspires.ftc.teamcode.Gains.POS_EPS;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public final class LimePoseSync {
    private LimePoseSync() {}

    public static int counter=0;
    public static boolean USE_INVERTED_FTC_COORDS = false;
    public static double X_OFFSET_IN = 0.0;
    public static double Y_OFFSET_IN = 0.0;
    public static double HEADING_OFFSET_DEG = -210.00;
    public static double MAX_STALENESS_MS = 120.0;
    public static int MIN_TAGS = 1;

    private static String lastState = "init";
    private static boolean lastApplied = false;
    private static int lastTagCount = 0;
    private static long lastStalenessMs = -1;
    private static double lastPedroX = Double.NaN;
    private static double lastPedroY = Double.NaN;
    private static double lastPedroHeadingDeg = Double.NaN;

    public static void sync(Follower follower, Limelight3A limelight, double turretAngleRads) {
        lastApplied = false;

        if (follower == null) { lastState = "no_follower"; return; }
        if (limelight == null) { lastState = "no_limelight"; return; }

        LLResult result = limelight.getLatestResult();
        if (result == null) { lastState = "no_result"; return; }

        lastTagCount = result.getBotposeTagCount();
        lastStalenessMs = result.getStaleness();

        if (!result.isValid()) { lastState = "invalid"; return; }
        if (lastStalenessMs > MAX_STALENESS_MS) { lastState = "stale"; return; }
        if (lastTagCount < MIN_TAGS) { lastState = "low_tags"; return; }

        // MT1 pose
        Pose3D bot = result.getBotpose();
        if (bot == null || bot.getPosition() == null || bot.getOrientation() == null) {
            lastState = "bad_pose3d";
            return;
        }

        double xIn = bot.getPosition().toUnit(DistanceUnit.INCH).x;
        double yIn = bot.getPosition().toUnit(DistanceUnit.INCH).y;
        double hRad = bot.getOrientation().getYaw(AngleUnit.RADIANS);

        if (!Double.isFinite(xIn) || !Double.isFinite(yIn) || !Double.isFinite(hRad)) {
            lastState = "nan_pose";
            return;
        }

        Pose ftcPose = USE_INVERTED_FTC_COORDS
                ? new Pose(xIn, yIn, hRad, InvertedFTCCoordinates.INSTANCE)
                : new Pose(xIn, yIn, hRad, FTCCoordinates.INSTANCE);

        Pose pedro = ftcPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

        Pose snapped = new Pose(
                pedro.getX() + X_OFFSET_IN,
                pedro.getY() + Y_OFFSET_IN,
                MathFunctions.normalizeAngle(pedro.getHeading() + turretAngleRads + Math.toRadians(HEADING_OFFSET_DEG))
        );

        lastPedroX = snapped.getX();
        lastPedroY = snapped.getY();
        lastPedroHeadingDeg = Math.toDegrees(snapped.getHeading());
        if (Math.abs(lastPedroX-follower.getPose().getX())>POS_EPS||Math.abs(lastPedroX-follower.getPose().getY())>POS_EPS){counter++; if (counter<30){;return;}} else {counter=0;}

        follower.setPose(snapped); // hard snap
        lastApplied = true;
        lastState = "applied";
    }

    public static String statusLine() {
        return String.format("MT1 state=%s applied=%s tags=%d stale=%dms",
                lastState, lastApplied, lastTagCount, lastStalenessMs);
    }

    public static String poseLine() {
        return String.format("MT1 snapped: x=%.2f y=%.2f h=%.1f",
                lastPedroX, lastPedroY, lastPedroHeadingDeg);
    }
}
