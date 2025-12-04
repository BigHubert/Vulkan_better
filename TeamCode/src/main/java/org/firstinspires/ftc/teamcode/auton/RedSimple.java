package org.firstinspires.ftc.teamcode.auton;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Simple", group = "Examples")
public class RedSimple extends OpMode {
    private Follower follower;
    private Timer pathTimer,opmodeTimer;
    private int pathState;
    private Limelight3A limelight;
    private DcMotor Shooter, Pusher, IntakeMotor, rotationMotor;
    private Servo hood;
    private final Pose startPose = new Pose(152, 82, Math.toRadians(90));
    private final Pose scorePose = new Pose(152, 140, Math.toRadians(55));
    private Path scorePreload;
    private static final double targetTx = 0.0;

    private static final double deadzone = 4.5;
    private static final double maxMotorPower = 0.25;
    private static final double rotationSpeed = 0.15;
    private static final double minMotorPower = 0.08;
    private double lastMotorPower = 0.0;

    private double getHoodPosition(double distance) {
        double pos = 0.25 + 0.15 * distance;
        return Range.clip(pos, 0.0, 1.0);
    }

    private static final double minShooterPower = 0.2;
    private static final double maxShooterPower = 0.9;
    private static final double minTa = 0.3;
    private static final double maxTa = 2.0;
    private static final double foundBonus = 0.2;
    private static final double noTargetPower = 0.0;
    private static final double llhieght = 0.25; // meters
    private static final double Theight = 1.5;   // meters
    private static final double llangle = 20.0;
    private double lastHoodPos = 0.5;   // any safe default
    private static final double hooddeadzone = 0.02;
    private boolean lastHadTarget = false;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(scorePreload);
                    setPathState(1);
                }
                break;
            case 1: {
                if (!follower.isBusy()) {
                    IntakeMotor.setPower(0.8);
                    Pusher.setPower(0.8);
                    setPathState(-1);
                }
            }
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        Shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
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
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        LLResult llResult = limelight.getLatestResult();
        double shooterPower = noTargetPower;
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        if (llResult != null && llResult.isValid()) {
            double tx = llResult.getTx();
            double ta = llResult.getTa();
            double error = targetTx - tx;
            double absError = Math.abs(error);
            double ty = llResult.getTy();


            if (ta > 0) {
                shooterPower = Range.clip(
                        maxShooterPower - (ta - minTa) *
                                (maxShooterPower - minShooterPower) / (maxTa - minTa),
                        minShooterPower,
                        maxShooterPower
                );
            }
            double totalAngle = Math.toRadians(llangle + ty);
            double distance = (Theight - llhieght) / Math.tan(totalAngle);
            double hoodPos = getHoodPosition(distance);
            if (Math.abs(hoodPos - lastHoodPos) > hooddeadzone) {
                hood.setPosition(hoodPos);
                lastHoodPos = hoodPos;
            }

            lastHadTarget = true;
            telemetry.addData("Hood", "Tracking target");


            telemetry.addData("Tx", tx);
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", ta);
            telemetry.addData("Error", error);
            telemetry.addData("Base ShooterPower", shooterPower);

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
                shooterPower += foundBonus;
                shooterPower = Range.clip(shooterPower, 0, 1.0); // Cap at 100%

                rotationMotor.setPower(0);
                lastMotorPower = 0;
                telemetry.addData("Status", "Centered");
                telemetry.addData("Target Found Bonus", foundBonus);
            }
        } else {
            shooterPower = noTargetPower;
            rotationMotor.setPower(0);
            lastMotorPower = 0;
            telemetry.addData("Status", "No Data");
            hood.setPosition(lastHoodPos);
            telemetry.addData("Hood", "No target - holding last position");
            lastHadTarget = false;
        }

        Shooter.setPower(shooterPower);

        telemetry.addData("Final Shooter Power", shooterPower);
        telemetry.update();
    }
}


