/*package org.firstinspires.ftc.teamcode.auton;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Cool auton", group = "Examples")
public class AutonSimple extends OpMode {

    private static TelemetryManager telemetryM;
    private static Follower follower;

    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(60, 12, Math.toRadians(90));
    private final Pose scorePose = new Pose(57, 91, Math.toRadians(130));
    private final Pose pickBall = new Pose(20.5, 35, Math.toRadians(180));
    private final Pose scorePose2 = new Pose(54, 93.5, Math.toRadians(130));
    private final Pose pickBall2 = new Pose(18.5, 59.5, Math.toRadians(180));
    private final Pose scorePose3 = new Pose(55, 95, Math.toRadians(130));
    private final Pose grabball1 = new Pose(37, 35, Math.toRadians(180));
    private final Pose grabball2 = new Pose(34, 60, Math.toRadians(180));

    // ------------------ PATHS ------------------
    private Path scorePreload;
    private PathChain grabBalls1, pickballs1, Score1, grabBall2, pickballs2, score2;

    // Store preview points for each path or chain
    private double[][] scorePreloadPoints;
    private double[][][] grabBalls1Points;
    private double[][][] pickballs1Points;
    private double[][][] Score1Points;
    private double[][][] grabBall2Points;
    private double[][][] pickballs2Points;
    private double[][][] score2Points;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose); // Must be set before building paths

        buildPaths(); // Now paths can safely be built using follower

        // Sample paths for preview
        scorePreloadPoints = safeSamplePath(scorePreload);
        grabBalls1Points = safeSampleChain(grabBalls1);
        pickballs1Points = safeSampleChain(pickballs1);
        Score1Points = safeSampleChain(Score1);
        grabBall2Points = safeSampleChain(grabBall2);
        pickballs2Points = safeSampleChain(pickballs2);
        score2Points = safeSampleChain(score2);

        Drawing.init();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void init_loop() {
        // Draw preview paths
        Drawing.drawPreviewPoints(scorePreloadPoints);
        Drawing.drawPreviewPoints(grabBalls1Points);
        Drawing.drawPreviewPoints(pickballs1Points);
        Drawing.drawPreviewPoints(Score1Points);
        Drawing.drawPreviewPoints(grabBall2Points);
        Drawing.drawPreviewPoints(pickballs2Points);
        Drawing.drawPreviewPoints(score2Points);

        Drawing.drawRobot(startPose);
        Drawing.sendPacket();

        telemetryM.update(telemetry);
    }

    private double[][] safeSamplePath(Path path) {
        if (path == null) return null;
        double[][] pts = path.getPanelsDrawingPoints();
        return (pts != null && pts.length > 1) ? pts : null;
    }

    private double[][][] safeSampleChain(PathChain chain) {
        if (chain == null || chain.size() == 0) return null;
        double[][][] result = new double[chain.size()][][];
        for (int i = 0; i < chain.size(); i++) {
            Path p = chain.getPath(i);
            result[i] = safeSamplePath(p);
        }
        return result;
    }

    private void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabBalls1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(64, 31, Math.toRadians(180)), pickBall))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickBall.getHeading())
                .build();

        pickballs1 = follower.pathBuilder()
                .addPath(new BezierLine(pickBall, grabball1))
                .setLinearHeadingInterpolation(pickBall.getHeading(), grabball1.getHeading())
                .build();

        Score1 = follower.pathBuilder()
                .addPath(new BezierCurve(pickBall, new Pose(75, 40, Math.toRadians(130)), scorePose2))
                .setLinearHeadingInterpolation(pickBall.getHeading(), scorePose2.getHeading())
                .build();

        grabBall2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose2, new Pose(65, 56, Math.toRadians(180)), pickBall2))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickBall2.getHeading())
                .build();

        pickballs2 = follower.pathBuilder()
                .addPath(new BezierLine(pickBall2, grabball2))
                .setLinearHeadingInterpolation(pickBall2.getHeading(), grabball2.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickBall2, new Pose(66, 53, Math.toRadians(130)), scorePose3))
                .setLinearHeadingInterpolation(pickBall2.getHeading(), scorePose3.getHeading())
                .build();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) { follower.followPath(grabBalls1, true); setPathState(2); }
                break;
            case 2:
                if (!follower.isBusy()) { follower.followPath(pickballs1, true); setPathState(3); }
                break;
            case 3:
                if (!follower.isBusy()) { follower.followPath(Score1, true); setPathState(4); }
                break;
            case 4:
                if (!follower.isBusy()) { follower.followPath(grabBall2, true); setPathState(5); }
                break;
            case 5:
                if (!follower.isBusy()) { follower.followPath(pickballs2, true); setPathState(6); }
                break;
            case 6:
                if (!follower.isBusy()) { follower.followPath(score2, true); setPathState(7); }
                break;
            case 7:
                if (!follower.isBusy()) setPathState(-1);
                break;
        }
    }

    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());
        telemetryM.debug("total heading:" + follower.getTotalHeading());
        telemetryM.update(telemetry);

        Drawing.drawDebug(follower);
        draw();
    }
}
class Drawing {
    public static final double ROBOT_RADIUS = 9;
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style("", "#3F51B5", 0.75);
    private static final Style historyLook = new Style("", "#4CAF50", 0.75);
    private static final Style previewLook = new Style("", "#9E9E9E", 0.50);

    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    // Draw preview from raw points (single path)
    public static void drawPreviewPoints(double[][] pts) {
        if (pts == null || pts.length < 2) return;
        panelsField.setStyle(previewLook);
        for (int i = 0; i < pts.length - 1; i++) {
            double x1 = pts[i][0], y1 = pts[i][1];
            double x2 = pts[i + 1][0], y2 = pts[i + 1][1];
            if (isValid(x1, y1) && isValid(x2, y2)) {
                panelsField.moveCursor(x1, y1);
                panelsField.line(x2, y2);
            }
        }
    }

    // Overloaded: chain of paths
    public static void drawPreviewPoints(double[][][] chainPts) {
        if (chainPts == null) return;
        for (double[][] pts : chainPts) {
            drawPreviewPoints(pts);
        }
    }

    private static boolean isValid(double x, double y) {
        return !Double.isNaN(x) && !Double.isNaN(y);
    }

    public static void drawDebug(Follower follower) {
        if (follower == null) return;

        try {
            Path current = follower.getCurrentPath();
            if (current != null) {
                drawPath(current, robotLook);
                Pose cp = follower.getPointFromPath(current.getClosestPointTValue());
                if (cp != null) drawRobot(cp, robotLook);
            }
        } catch (Exception ignored) {}

        try { drawPoseHistory(follower.getPoseHistory(), historyLook); } catch (Exception ignored) {}
        try { drawRobot(follower.getPose(), historyLook); } catch (Exception ignored) {}

        panelsField.update();
    }

    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || !isValid(pose.getX(), pose.getY())) return;

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);

        double x1 = pose.getX() + v.getXComponent() / 2;
        double y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent();
        double y2 = pose.getY() + v.getYComponent();

        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    public static void drawPath(Path path, Style style) {
        if (path == null) return;
        double[][] pts = path.getPanelsDrawingPoints();
        if (pts == null || pts.length < 2) return;

        panelsField.setStyle(style);
        for (int i = 0; i < pts.length - 1; i++) {
            double x1 = pts[i][0], y1 = pts[i][1];
            double x2 = pts[i + 1][0], y2 = pts[i + 1][1];
            if (isValid(x1, y1) && isValid(x2, y2)) {
                panelsField.moveCursor(x1, y1);
                panelsField.line(x2, y2);
            }
        }
    }

    public static void drawPath(PathChain chain, Style style) {
        if (chain == null) return;
        for (int i = 0; i < chain.size(); i++) {
            drawPath(chain.getPath(i), style);
        }
    }

    public static void drawPoseHistory(PoseHistory hist, Style style) {
        if (hist == null) return;
        panelsField.setStyle(style);
        double[] xs = hist.getXPositionsArray();
        double[] ys = hist.getYPositionsArray();
        if (xs == null || ys == null || xs.length < 2 || ys.length < 2) return;
        for (int i = 0; i < xs.length - 1; i++) {
            if (isValid(xs[i], ys[i]) && isValid(xs[i + 1], ys[i + 1])) {
                panelsField.moveCursor(xs[i], ys[i]);
                panelsField.line(xs[i + 1], ys[i + 1]);
            }
        }
    }

    public static void sendPacket() {
        panelsField.update();
    }
}
*/