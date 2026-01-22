package org.firstinspires.ftc.teamcode.pedroPathing;
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
@Autonomous(name = "Red Close Ramp 3 clears", group = "Red Ramp")
public class RedCloseRamp extends OpMode {
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
        CLEAR_RAMP4,
        CLEAR_RAMP5,
        CLEAR_RAMP6,
        CLEAR_RAMP7,
        CLEAR_RAMP8,
        CLEAR_RAMP9,
        LAUNCH_CLEARANCE
    }
    PathState pathState;
    private final Pose startPose = new Pose(124.5, 122, Math.toRadians(36));
    private final Pose shootPose = new Pose(87, 86, Math.toRadians(44));
    private final Pose Startofballs1 = new Pose(100,85, Math.toRadians(0));
    private final Pose EndofBalls1 = new Pose(124,85, Math.toRadians(0));
    private final Pose Startofballs2 = new Pose(100,61, Math.toRadians(0));
    private final Pose EndofBalls2 = new Pose(127,61, Math.toRadians(0));
    private final Pose Startofballs3 = new Pose(100,36, Math.toRadians(0));
    private final Pose EndofBalls3 = new Pose(127,36, Math.toRadians(0));
    private final Pose ClearRamp1 = new Pose(100,75.25, Math.toRadians(0));
    private final Pose ClearRamp2 = new Pose(125,75.25, Math.toRadians(0));
    private final Pose ClearRamp3 = new Pose(100,78, Math.toRadians(0));
    private final Pose LaunchClearance = new Pose(100,70, Math.toRadians(90));

    private PathChain diveStartPosShootPos,grabballs1,pickballs1,shoot1,grabballs2,pickballs2,shoot2,grabballs3,pickballs3,shoot3,clearramp1,clearramp2,clearramp3,clearramp4,launchclearance;
    public void buildPaths() {
        diveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        grabballs1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,Startofballs1))
                .setLinearHeadingInterpolation(shootPose.getHeading(),Startofballs1.getHeading())
                .build();
        pickballs1 = follower.pathBuilder()
                .addPath(new BezierLine(Startofballs1,EndofBalls1))
                .setLinearHeadingInterpolation(Startofballs1.getHeading(), EndofBalls1.getHeading())
                .build();
        shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(ClearRamp2,shootPose))
                .setLinearHeadingInterpolation(ClearRamp2.getHeading(), shootPose.getHeading())
                .build();
        grabballs2 = follower.pathBuilder()
                .addPath(new BezierLine(ClearRamp3,Startofballs2))
                .setLinearHeadingInterpolation(ClearRamp3.getHeading(), Startofballs2.getHeading())
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
                .addPath(new BezierLine(EndofBalls1,ClearRamp1))
                .setLinearHeadingInterpolation(EndofBalls1.getHeading(), ClearRamp1.getHeading())
                .build();
        clearramp2 = follower.pathBuilder()
                .addPath(new BezierLine(ClearRamp1,ClearRamp2))
                .setLinearHeadingInterpolation(ClearRamp1.getHeading(), ClearRamp2.getHeading())
                .build();
        clearramp3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,ClearRamp2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), ClearRamp2.getHeading())
                .build();
        clearramp4 = follower.pathBuilder()
                .addPath(new BezierLine(ClearRamp2,ClearRamp3))
                .setLinearHeadingInterpolation(ClearRamp2.getHeading(), ClearRamp3.getHeading())
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
                    Shooter.setPower(0.675);
                }
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) {
                    setPathState(PathState.GRAB_BALLS_1);
                    Shooter.setPower(0.675);
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
                    setPathState(PathState.CLEAR_RAMP1);
                    IntakeMotor.setPower(0.9);
                }
                break;
            case CLEAR_RAMP1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(clearramp1, true);
                    setPathState(PathState.CLEAR_RAMP2);
                    Shooter.setPower(0);
                    IntakeMotor.setPower(0);
                    Pusher.setPower(0);
                }
                break;
            case CLEAR_RAMP2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(clearramp2, true);
                    setPathState(PathState.SHOOT1);
                }
                break;
            case SHOOT1 :
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5){
                    follower.followPath(shoot1, true);
                    setPathState(PathState.SHOOT_BALLS_1);
                    limelight.pipelineSwitch(0);
                    Shooter.setPower(0.675);
                    IntakeMotor.setPower(0);
                }
                break;
            case SHOOT_BALLS_1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(PathState.CLEAR_RAMP3);
                    Shooter.setPower(0.675);
                    IntakeMotor.setPower(0.9);
                    Pusher.setPower(0.9);
                }
                break;
            case CLEAR_RAMP3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) {
                    follower.followPath(clearramp3, true);
                    setPathState(PathState.CLEAR_RAMP4);
                    limelight.pipelineSwitch(1);
                    Shooter.setPower(0);
                    IntakeMotor.setPower(0);
                    Pusher.setPower(0);
                }
                break;
            case CLEAR_RAMP4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(clearramp4, true);
                    setPathState(PathState.GRAB_BALLS_2);
                }
                break;
            case GRAB_BALLS_2 :
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(grabballs2, true);
                    setPathState(PathState.PICK_BALLS2);
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
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(shoot2, true);
                    setPathState(PathState.SHOOT_BALLS_2);
                    limelight.pipelineSwitch(0);
                    Shooter.setPower(0.675);
                    IntakeMotor.setPower(0);

                }
                break;
            case SHOOT_BALLS_2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) {
                    setPathState(PathState.LAUNCH_CLEARANCE);
                    Shooter.setPower(0.675);
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
        limelight.pipelineSwitch(0);
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