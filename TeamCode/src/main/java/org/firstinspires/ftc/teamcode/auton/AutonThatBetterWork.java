package org.firstinspires.ftc.teamcode.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Autonomous(name = "Auton That better work Blue", group = "Examples")
public class AutonThatBetterWork extends OpMode {
    private Follower follower;
    private Timer pathTimer,opModeTimer;
    public enum PathState {
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        //must declare each new path
        GRAB_BALLS_1,
        STOP,
        PICK_BALLS1
    }
    PathState pathState;
    private final Pose startPose = new Pose(60, 10, Math.toRadians(90));
    private final Pose shootPose = new Pose(60, 90, Math.toRadians(40));
    private final Pose Startofballs1 = new Pose(35,35, Math.toRadians(0));
    private final Pose EndofBalls1 = new Pose(10,35, Math.toRadians(0));
    private PathChain diveStartPosShootPos,grabballs1,pickballs1;
    public void buildPaths() {
        diveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        grabballs1 =follower.pathBuilder()
                .addPath(new BezierLine(shootPose,Startofballs1))
                .setLinearHeadingInterpolation(shootPose.getHeading(),Startofballs1.getHeading())
                .build();
        pickballs1 = follower.pathBuilder()
                .addPath(new BezierLine(Startofballs1,EndofBalls1))
                .setLinearHeadingInterpolation(Startofballs1.getHeading(), EndofBalls1.getHeading())
                .build();
    }
    public void statePathUpdate() {
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(diveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                //make Shooter shoot
                if (!follower.isBusy() /*&& pathTimer.getElapsedTimeSeconds() > 5*/) {
                    setPathState(PathState.GRAB_BALLS_1);
                    telemetry.addLine("Done Path 1");
                }
                break;
            case GRAB_BALLS_1:
                if (!follower.isBusy() /*&& pathTimer.getElapsedTimeSeconds() > 5*/) {
                    follower.followPath(grabballs1, true);
                    setPathState(PathState.PICK_BALLS1);
                    telemetry.addLine("Done Path 2");
                }
                break;
            case PICK_BALLS1 :
                if (!follower.isBusy() /*&& pathTimer.getElapsedTimeSeconds() > 5*/) {
                    follower.followPath(pickballs1, true);
                    setPathState(PathState.STOP);
                    telemetry.addLine("Done Path 3");
                }

            case STOP:
                if (!follower.isBusy()){
                    follower.holdPoint(follower.getPose());
                    telemetry.addLine("Stopped");
                }
                break;

            default:
                telemetry.addLine("No State Command");
                break;
        }
    }
    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        telemetry.addData("paths state", pathState.toString());
        telemetry.addData("x",follower.getPose().getX());
        telemetry.addData("y",follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
    }
}
