package org.firstinspires.ftc.teamcode.auton;
import com.bylazar.field.FieldManager;
import com.bylazar.field.FieldPresetParams;
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

@Autonomous(name = "Vulkan Auton Blue", group = "Examples")
public class Pedrotest extends OpMode {

    static TelemetryManager telemetryM;
    static PoseHistory poseHistory;
    private static Follower follower;


    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private boolean allowUpdate = false;

    // Offset compensation: add 72 to X and Y to match visualization offset of (-72, -72)
    private final Pose startPose = new Pose(132, 82, Math.toRadians(90));
    private final Pose scorePose = new Pose(129, 140, Math.toRadians(130));
    private final Pose pickBall = new Pose(92.5, 107, Math.toRadians(180));
    private final Pose scorePose2 = new Pose(126, 165.5, Math.toRadians(130));
    private final Pose pickBall2 = new Pose(90.5, 131.5, Math.toRadians(180));
    private final Pose scorePose3 = new Pose(127, 167, Math.toRadians(130));
    private final Pose grabball1 = new Pose(109, 107, Math.toRadians(180));
    private final Pose grabball2 = new Pose(106, 132, Math.toRadians(180));
    private final Pose endPose = new Pose(127, 206, Math.toRadians(270));
    private PathChain scorePreload,grabBalls1, pickballs1, Score1, grabBall2, pickballs2, score2,finish;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        poseHistory = follower.getPoseHistory();
        Drawing.init();
        // CRITICAL: Set starting pose BEFORE building paths
        // Make sure this matches your robot's ACTUAL starting position on the field
        follower.setStartingPose(startPose);
        buildPaths();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    public static void drawOnlyCurrent() {
        try {
            org.firstinspires.ftc.teamcode.auton.Drawing.drawRobot(follower.getPose());
            org.firstinspires.ftc.teamcode.auton.Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    public static void draw() {
        org.firstinspires.ftc.teamcode.auton.Drawing.drawDebug(follower);
    }

    private void drawAllPaths() {
        Style pathStyle = new Style("", "#FF9800", 0.5);

        // Draw all paths in sequence
        Drawing.drawPath(scorePreload, pathStyle);
        Drawing.drawPath(grabBalls1, pathStyle);
        Drawing.drawPath(pickballs1, pathStyle);
        Drawing.drawPath(Score1, pathStyle);
        Drawing.drawPath(grabBall2, pathStyle);
        Drawing.drawPath(pickballs2, pathStyle);
        Drawing.drawPath(score2, pathStyle);
        Drawing.drawPath(finish, pathStyle);
    }

    public void init_loop(){
        if (follower != null) {

            // Draw all paths that will be followed during autonomous
            drawAllPaths();

            // Draw the current robot pose
            drawOnlyCurrent();
        }

        telemetryM.update(telemetry);
    }

    private void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        grabBalls1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(64, 31, Math.toRadians(180)), pickBall))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickBall.getHeading())
                .build();

        pickballs1 = follower.pathBuilder()
                .addPath(new BezierLine(pickBall, grabball1))
                .setLinearHeadingInterpolation(pickBall.getHeading(), grabball1.getHeading())
                .build();

        Score1 = follower.pathBuilder()
                .addPath(new BezierCurve(grabball1, new Pose(75, 40, Math.toRadians(130)), scorePose2))
                .setLinearHeadingInterpolation(grabball1.getHeading(), scorePose2.getHeading())
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
                .addPath(new BezierCurve(grabball2, new Pose(66, 53, Math.toRadians(130)), scorePose3))
                .setLinearHeadingInterpolation(grabball2.getHeading(), scorePose3.getHeading())
                .build();
        finish = follower.pathBuilder()
                .addPath(new BezierLine(scorePose3, endPose))
                .setLinearHeadingInterpolation(scorePose3.getHeading(), endPose.getHeading())
                .build();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);

                setPathState(1);
                break;

            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(grabBalls1, true);
                    setPathState(2);
                }
                break;

            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(pickballs1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(Score1, true);
                    setPathState(4);
                }
                break;

            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(grabBall2, true);
                    setPathState(5);
                }
                break;

            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(pickballs2, true);
                    setPathState(6);
                }
                break;

            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(score2, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(finish, true);
                    setPathState(8);
                }
                break;

            case 8:
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
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

        // Let localizer settle
        follower.update();
        Pose actualStartPose = follower.getPose();

        // Calculate offset between desired and actual
        double xOffset = startPose.getX() - actualStartPose.getX();
        double yOffset = startPose.getY() - actualStartPose.getY();
        double headingOffset = startPose.getHeading() - actualStartPose.getHeading();

        telemetryM.debug("X Offset: " + xOffset);
        telemetryM.debug("Y Offset: " + yOffset);
        telemetryM.debug("Heading Offset: " + Math.toDegrees(headingOffset));

        // If the offset is large, you might need to rebuild paths
        // Or just accept the actual starting pose and adjust your field coordinates

        follower.setStartingPose(actualStartPose); // Use actual pose
        allowUpdate = true;
        setPathState(0);
    }


    @Override
    public void loop() {
        if (allowUpdate) {
            follower.update();
            autonomousPathUpdate();
        }
        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + Math.toDegrees(follower.getPose().getHeading()));
        telemetryM.debug("total heading:" + Math.toDegrees(follower.getTotalHeading()));
        telemetryM.debug("pathState:" + pathState);
        telemetryM.debug("isBusy:" + follower.isBusy());
        telemetryM.update(telemetry);

        Drawing.drawDebug(follower);
        draw();
    }
}

class Drawing {
    public static final double ROBOT_RADIUS = 9; // woah
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style("", "#3F51B5", 0.75);
    private static final Style historyLook = new Style("", "#4CAF50", 0.75);

    /**
     * This prepares Panels Field for using Pedro Offsets
     */


    public static void init() {
        FieldPresetParams customPreset = new FieldPresetParams();
        panelsField.setOffsets(customPreset);
    }

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashbaordDrawingHandler class is not needed.
     *
     * @param follower Pedro Follower instance.
     */
    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook);
        drawRobot(follower.getPose(), historyLook);
        sendPacket();
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose  the Pose to draw the robot at
     * @param style the parameters used to draw the robot with
     */
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    /**
     * This draws a robot at a specified Pose. The heading is represented as a line.
     *
     * @param pose the Pose to draw the robot at
     */
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    /**
     * This draws a Path with a specified look.
     * FIXED: Draws a continuous curved line by explicitly connecting each point to the next
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    public static void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();

        // Validate and fix NaN values
        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }

        if (points[0].length < 2) {
            return; // Need at least 2 points to draw a line
        }

        panelsField.setStyle(style);

        // Determine step size for curves
        int step = 1;
        if (points[0].length > 100) {
            step = Math.max(1, points[0].length / 150);
        }

        // Draw continuous path: connect point i to point i+1
        // Start at first point
        double prevX = points[0][0];
        double prevY = points[1][0];

        // Draw line segments connecting consecutive sampled points
        for (int i = step; i < points[0].length; i += step) {
            double currX = points[0][i];
            double currY = points[1][i];

            // Move to previous point, then draw line to current point
            panelsField.moveCursor(prevX, prevY);
            panelsField.line(currX, currY);

            // Update previous point for next iteration
            prevX = currX;
            prevY = currY;
        }

        // Complete the path to the very last point if we didn't already
        int lastIndex = points[0].length - 1;
        if (lastIndex % step != 0) {
            panelsField.moveCursor(prevX, prevY);
            panelsField.line(points[0][lastIndex], points[1][lastIndex]);
        }
    }

    /**
     * This draws all the Paths in a PathChain with a
     * specified look.
     *
     * @param pathChain the PathChain to draw
     * @param style     the parameters used to draw the PathChain with
     */
    public static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     * @param style       the parameters used to draw the pose history with
     */
    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
        panelsField.setStyle(style);

        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {
            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     **/
    public static void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, historyLook);
    }

    /**
     * This tries to send the current packet to FTControl Panels.
     **/
    public static void sendPacket() {
        panelsField.update();
    }
}