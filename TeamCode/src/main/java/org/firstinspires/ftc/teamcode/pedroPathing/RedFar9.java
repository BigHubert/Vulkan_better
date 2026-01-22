package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Autonomous(name = "Red Far start, close shoot 9 no clear", group = "Red 9")
public class RedFar9 extends OpMode {
    private static final double targetTx = 0.0;
    private static final double deadzone = 4.5;
    private static final double maxMotorPower = 0.25;
    private static final double rotationSpeed = 0.15;
    private static final double minMotorPower = 0.08;
    private static final double foundBonus = 0.10;
    private static final double noTargetPower = 0.0;
    private static final double llhieght = 0.25; // meters
    private static final double Theight = 1.5;   // meters
    private static final double llangle = 20.0;
    private double lastHoodPos = 0.5;   // any safe default
    private static final double hooddeadzone = 0.02;

    private boolean lastHadTarget = false;
    private DcMotor rotationMotor;
    private Limelight3A limelight;
    private DcMotor Shooter;
    private DcMotor IntakeMotor, Pusher;
    private Servo hood;
    private double lastMotorPower = 0.0;

    // RPM calculation variables
    private int lastShooterPosition = 0;
    private long lastTime;

    private double getHoodPosition(double distance) {
        // Inverted: farther = moves up (lower position value), close = moves down (higher position value)
        // Using max distance as reference - adjust these values as needed
        double maxDistance = 5.0; // Adjust based on your max expected distance
        double pos = 0.75 - 0.15 * (distance / maxDistance);
        return Range.clip(pos, 0.0, 1.0);
    }

    private Follower follower;
    private Timer pathTimer,opModeTimer;
    public enum PathState {
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        //must declare each new path
        GRAB_BALLS_1,
        STOP,
        PICK_BALLS1,
        SHOOT1,
        GRAB_BALLS_2,
        PICK_BALLS2,
        SHOOT2,
        SHOOT_BALLS_1,
        SHOOT_BALLS_2,
        SHOOT_BALLS_3,
        SHOOT3,
        GRAB_BALLS_3,
        PICK_BALLS3,
        CLEAR_RAMP1,
        CLEAR_RAMP2,
        CLEAR_RAMP3,
        LAUNCH_CLEARANCE
    }
    PathState pathState;
    private final Pose startPose = new Pose(87, 9, Math.toRadians(90));
    private final Pose shootPose = new Pose(85, 15, Math.toRadians(55));
    private final Pose Startofballs1 = new Pose(100,33, Math.toRadians(0));
    private final Pose EndofBalls1 = new Pose(128,33, Math.toRadians(0));
    private final Pose Startofballs2 = new Pose(100,57, Math.toRadians(0));
    private final Pose EndofBalls2 = new Pose(128,57, Math.toRadians(0));
    private final Pose Startofballs3 = new Pose(100,85, Math.toRadians(0));
    private final Pose EndofBalls3 = new Pose(126,85, Math.toRadians(0));
    private final Pose ClearRamp1 = new Pose(100,74, Math.toRadians(0));
    private final Pose ClearRamp2 = new Pose(124,74, Math.toRadians(0));
    private final Pose LaunchClearance = new Pose(100,30, Math.toRadians(90));
    private PathChain diveStartPosShootPos,grabballs1,pickballs1,shoot1,grabballs2,pickballs2,shoot2,grabballs3,pickballs3,shoot3,clearramp1,clearramp2,clearramp3,launchclearance;
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
        shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(EndofBalls1,shootPose))
                .setLinearHeadingInterpolation(EndofBalls1.getHeading(), shootPose.getHeading())
                .build();
        grabballs2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,Startofballs2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Startofballs2.getHeading())
                .build();
        pickballs2 = follower.pathBuilder()
                .addPath(new BezierLine(Startofballs2,EndofBalls2))
                .setLinearHeadingInterpolation(Startofballs2.getHeading(), EndofBalls2.getHeading())
                .build();
        shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(EndofBalls2,shootPose))
                .setLinearHeadingInterpolation(EndofBalls2.getHeading(), shootPose.getHeading())
                .build();
        grabballs3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,Startofballs3))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Startofballs3.getHeading())
                .build();
        pickballs3 = follower.pathBuilder()
                .addPath(new BezierLine(Startofballs3,EndofBalls3))
                .setLinearHeadingInterpolation(Startofballs3.getHeading(), EndofBalls3.getHeading())
                .build();
        shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(EndofBalls3,shootPose))
                .setLinearHeadingInterpolation(EndofBalls3.getHeading(), shootPose.getHeading())
                .build();
        clearramp1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,ClearRamp1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), ClearRamp1.getHeading())
                .build();
        clearramp2 = follower.pathBuilder()
                .addPath(new BezierLine(ClearRamp1,ClearRamp2))
                .setLinearHeadingInterpolation(ClearRamp1.getHeading(), ClearRamp2.getHeading())
                .build();
        clearramp3 = follower.pathBuilder()
                .addPath(new BezierLine(ClearRamp2,ClearRamp1))
                .setLinearHeadingInterpolation(ClearRamp2.getHeading(), ClearRamp1.getHeading())
                .build();
        launchclearance = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,LaunchClearance))
                .setLinearHeadingInterpolation(shootPose.getHeading(), LaunchClearance.getHeading())
                .build();
    }
    public void statePathUpdate() {
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                if (!follower.isBusy()) {
                    follower.followPath(diveStartPosShootPos, true);
                    setPathState(PathState.SHOOT_PRELOAD);
                    limelight.pipelineSwitch(1);
                    Shooter.setPower(0.8);
                }
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.25) {
                    setPathState(PathState.GRAB_BALLS_1);
                    Shooter.setPower(0.8);
                    IntakeMotor.setPower(0.9);
                    Pusher.setPower(0.9);
                }
                break;
            case GRAB_BALLS_1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(grabballs1, true);
                    setPathState(PathState.PICK_BALLS1);
                    Shooter.setPower(0);
                    IntakeMotor.setPower(0);
                    Pusher.setPower(0);
                }
                break;
            case PICK_BALLS1 :
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(pickballs1, true);
                    setPathState(PathState.SHOOT1);
                    IntakeMotor.setPower(0.9);
                }
                break;
            case SHOOT1 :
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1){
                    follower.followPath(shoot1, true);
                    setPathState(PathState.SHOOT_BALLS_1);
                    limelight.pipelineSwitch(1);
                    Shooter.setPower(0.85);
                    IntakeMotor.setPower(0);
                }
                break;
            case SHOOT_BALLS_1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.25) {
                    setPathState(PathState.GRAB_BALLS_2);
                    Shooter.setPower(0.875);
                    IntakeMotor.setPower(0.9);
                    Pusher.setPower(0.9);
                }
                break;
            case GRAB_BALLS_2 :
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(grabballs2, true);
                    setPathState(PathState.PICK_BALLS2);
                    Shooter.setPower(0);
                    IntakeMotor.setPower(0);
                    Pusher.setPower(0);
                }
                break;
            case PICK_BALLS2 :
                if (!follower.isBusy()) {
                    follower.followPath(pickballs2, true);
                    setPathState(PathState.SHOOT2);
                    IntakeMotor.setPower(0.9);
                }
                break;
            case SHOOT2 :
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(shoot2, true);
                    setPathState(PathState.SHOOT_BALLS_2);
                    limelight.pipelineSwitch(1);
                    Shooter.setPower(0.835);
                    IntakeMotor.setPower(0);

                }
                break;
            case SHOOT_BALLS_2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.75) {
                    setPathState(PathState.LAUNCH_CLEARANCE);
                    Shooter.setPower(0.845);
                    IntakeMotor.setPower(0.9);
                    Pusher.setPower(0.9);
                }
                break;
            case LAUNCH_CLEARANCE:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(launchclearance, true);
                    setPathState(PathState.STOP);
                    limelight.pipelineSwitch(1);
                    Shooter.setPower(0);
                    IntakeMotor.setPower(0);
                    Pusher.setPower(0);
                }
                break;
            case STOP:
                if (!follower.isBusy()&& pathTimer.getElapsedTimeSeconds() > 3){
                    follower.holdPoint(follower.getPose());
                    telemetry.addLine("Stopped");
                    limelight.pipelineSwitch(1);
                    Shooter.setPower(0);
                    IntakeMotor.setPower(0);
                    Pusher.setPower(0);
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
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hood = hardwareMap.get(Servo.class, "hood");
        hood.setDirection(Servo.Direction.REVERSE);


        IntakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        Pusher = hardwareMap.get(DcMotor.class, "Pusher");

        Pusher.setDirection(DcMotorSimple.Direction.FORWARD);
        IntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Pusher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        Shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
        lastShooterPosition = Shooter.getCurrentPosition();
        lastTime = System.nanoTime();

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
