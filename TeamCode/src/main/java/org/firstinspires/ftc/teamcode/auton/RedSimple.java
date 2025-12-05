package org.firstinspires.ftc.teamcode.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Autonomous(name = "Vulkan Auton Red Far", group = "Examples")
public class RedSimple extends OpMode {
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
        SHOOT2
    }
    PathState pathState;
    private final Pose startPose = new Pose(80, 10, Math.toRadians(90));
    private final Pose shootPose = new Pose(80, 90, Math.toRadians(45));
    private final Pose Startofballs1 = new Pose(105,35, Math.toRadians(0));
    private final Pose EndofBalls1 = new Pose(130,35, Math.toRadians(0));
    private final Pose Startofballs2 = new Pose(105,60, Math.toRadians(0));
    private final Pose EndofBalls2 = new Pose(126,60, Math.toRadians(0));
    private PathChain diveStartPosShootPos,grabballs1,pickballs1,shoot1,grabballs2,pickballs2,shoot2;
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
    }
    public void statePathUpdate() {
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(diveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    setPathState(PathState.GRAB_BALLS_1);
                    telemetry.addLine("Done Path 1");
                    Shooter.setPower(0.7);
                    IntakeMotor.setPower(0.9);
                    Pusher.setPower(0.9);
                }
                break;
            case GRAB_BALLS_1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(grabballs1, true);
                    setPathState(PathState.PICK_BALLS1);
                    telemetry.addLine("Done Path 2");
                    Shooter.setPower(0);
                    IntakeMotor.setPower(0);
                    Pusher.setPower(0);
                }
                break;
            case PICK_BALLS1 :
                if (!follower.isBusy() /*&& pathTimer.getElapsedTimeSeconds() > 5*/) {
                    follower.followPath(pickballs1, true);
                    setPathState(PathState.SHOOT1);
                    telemetry.addLine("Done Path 3");
                }
                break;
            case SHOOT1 :
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){
                    follower.followPath(shoot1, true);
                    setPathState(PathState.GRAB_BALLS_2);
                    telemetry.addLine("Done Path 4");
                }
                break;
            case GRAB_BALLS_2 :
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(grabballs2, true);
                    setPathState(PathState.PICK_BALLS2);
                    telemetry.addLine("Done Path 5");
                }
                break;
            case PICK_BALLS2 :
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(pickballs2, true);
                    setPathState(PathState.SHOOT2);
                    telemetry.addLine("Done Path 6");
                }
                break;
            case SHOOT2 :
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(shoot2, true);
                    setPathState(PathState.STOP);
                    telemetry.addLine("Done Path 7");
                }
                break;
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
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {

            double tx = llResult.getTx();
            double ta = llResult.getTa();
            double error = targetTx - tx;
            double absError = Math.abs(error);
            double ty = llResult.getTy();

            double totalAngle = Math.toRadians(llangle + ty);
            double distance = (Theight - llhieght) / Math.tan(totalAngle);

            double hoodPos = getHoodPosition(distance);

            if (Math.abs(hoodPos - lastHoodPos) > hooddeadzone) {
                hood.setPosition(hoodPos);
                lastHoodPos = hoodPos;
            }

            lastHadTarget = true;

            if (absError > deadzone) {
                double motorPower = error * rotationSpeed;
                motorPower = Range.clip(motorPower, -maxMotorPower, maxMotorPower);

                if (Math.abs(motorPower) > 0 && Math.abs(motorPower) < minMotorPower) {
                    motorPower = Math.signum(motorPower) * minMotorPower;
                }

                rotationMotor.setPower(motorPower);
                lastMotorPower = motorPower;

                telemetry.addData("Status", "Tracking");
                telemetry.addData("Motor Power", motorPower);
            } else {

                rotationMotor.setPower(0);
                lastMotorPower = 0;

                telemetry.addData("Status", "Centered");
                telemetry.addData("Target Found Bonus", foundBonus);
            }

        } else {
            rotationMotor.setPower(0);
            lastMotorPower = 0;
            telemetry.addData("Status", "No Data");
            hood.setPosition(lastHoodPos);
            telemetry.addData("Hood", "No target - holding last position");
            lastHadTarget = false;
        }
    }
}