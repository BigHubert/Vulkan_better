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
@Autonomous(name = "Blue Far Start", group = "Examples")
public class BlueSimple extends OpMode {
    private static final double targetTx = 0.0;
    private static final double deadzone = 5.5;
    private static final double rotationSpeed = 0.15;
    private static final double maxMotorPower = 0.25;
    private static final double minMotorPower = 0.08;
    private static final double hoodDeadzone = 0.02;
    private int lastShooterPosition = 0;
    private long lastTime;
    private double lastHoodPos = 0.5;
    private double smoothedTA = 1.0;
    private Servo hood;
    private Limelight3A limelight;
    private DcMotor rotationMotor, shooter,intake, pusher;
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
    private final Pose shootPose = new Pose(60, 20, Math.toRadians(40));
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
                intake.setPower(0.9);
                pusher.setPower(0.9);
                shooter.setPower(1);
                follower.holdPoint(follower.getPose());
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    setPathState(PathState.GRAB_BALLS_1);
                    intake.setPower(0);
                    pusher.setPower(0);
                    shooter.setPower(0);
                    telemetry.addLine("Done Path 1");
                }
                break;
            case GRAB_BALLS_1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(grabballs1, true);
                    setPathState(PathState.PICK_BALLS1);
                    telemetry.addLine("Done Path 2");
                }
                break;
            case PICK_BALLS1 :
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
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
        rotationMotor = hardwareMap.get(DcMotor .class, "rotationMotor");
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hood = hardwareMap.get(Servo .class, "hood");
        hood.setDirection(Servo.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "IntakeMotor");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        pusher = hardwareMap.get(DcMotor.class, "Pusher");

        shooter = hardwareMap.get(DcMotor.class, "Shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pusher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A .class, "Limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        lastShooterPosition = shooter.getCurrentPosition();
        lastTime = System.nanoTime();

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
        intake.setPower(gamepad1.a ? 0.9 : 0);
        pusher.setPower(gamepad1.b ? 0.9 : 0);


        LLResult result = limelight.getLatestResult();

        boolean targetVisible = false;
        double tx = 0;
        double ta = 1.0;


        double shooterPower = 0;
        double targetRPM = 0;

        if (result != null && result.isValid()) {
            targetVisible = true;
            tx = result.getTx();
            ta = result.getTa();


            smoothedTA = (smoothedTA * 0.8) + (ta * 0.2);


            double error = targetTx - tx;
            if (Math.abs(error) > deadzone) {
                double power = Range.clip(error * rotationSpeed, -maxMotorPower, maxMotorPower);
                if (Math.abs(power) < minMotorPower)
                    power = Math.signum(power) * minMotorPower;
                rotationMotor.setPower(power);
            } else {
                rotationMotor.setPower(0);
            }


            double hoodPos = Range.clip(
                    0.5 + (tx * 0.01),
                    0.0,
                    1.0
            );

            if (Math.abs(hoodPos - lastHoodPos) > hoodDeadzone) {
                hood.setPosition(hoodPos);
                lastHoodPos = hoodPos;
            }


        } else {
            rotationMotor.setPower(0);
        }


        int currentPos = shooter.getCurrentPosition();
        long now = System.nanoTime();

        int deltaPos = currentPos - lastShooterPosition;
        long deltaTime = now - lastTime;

        double ticksPerSec = (deltaPos / (deltaTime / 1e9));
        double rpm = ticksPerSec * 60.0 / 28.0;

        lastShooterPosition = currentPos;
        lastTime = now;
        boolean shooterReady = targetVisible && Math.abs(rpm - targetRPM) <= 1000;
    }
}
