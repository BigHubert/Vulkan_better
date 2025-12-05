package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Vulkan Tele Blue", group = "TeleOp")
public class turretBlue extends LinearOpMode {

    private static final double targetTx = 0.0;
    private static final double deadzone = 4.5;
    private static final double maxMotorPower = 0.25;
    private static final double rotationSpeed = 0.15;
    private static final double minMotorPower = 0.08;
    private static final double minShooterPower = 0.4;
    private static final double maxShooterPower = 0.9;
    private static final double minTa = 0.1;
    private static final double maxTa = 5.0;

    // Additional power when target is found and centered
    private static final double foundBonus = 0.10;
    private static final double noTargetPower = 0.0;
    private static final double rpmTolerance = 3000.0;
    private static final double llhieght = 0.25; // meters
    private static final double Theight = 1.5;   // meters
    private static final double llangle = 20.0;
    private double lastHoodPos = 0.5;   // any safe default
    private static final double hooddeadzone = 0.02;

    private boolean lastHadTarget = false;
    double frontLeftPower = (0.15);
    double frontRightPower = (0.15);
    double backLeftPower = (0.15);
    double backRightPower = (0.15);

    private DcMotor rotationMotor;
    private Limelight3A limelight;
    private DcMotor Shooter;
    private DcMotor motor_rf, motor_lf, motor_rb, motor_lb, IntakeMotor, Pusher;
    private Servo hood;

    private double lastMotorPower = 0.0;

    // RPM calculation variables
    private int lastShooterPosition = 0;
    private long lastTime;
    private RevBlinkinLedDriver light;

    private double getHoodPosition(double distance) {
        // Inverted: farther = moves up (lower position value), close = moves down (higher position value)
        // Using max distance as reference - adjust these values as needed
        double maxDistance = 5.0; // Adjust based on your max expected distance
        double pos = 0.75 - 0.15 * (distance / maxDistance);
        return Range.clip(pos, 0.0, 1.0);
    }

    @Override
    public void runOpMode() {

        // Map hardware
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hood = hardwareMap.get(Servo.class, "hood");
        hood.setDirection(Servo.Direction.REVERSE);

        motor_lf = hardwareMap.get(DcMotor.class, "motor_lf");
        motor_rf = hardwareMap.get(DcMotor.class, "motor_rf");
        motor_lb = hardwareMap.get(DcMotor.class, "motor_lb");
        motor_rb = hardwareMap.get(DcMotor.class, "motor_rb");

        IntakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        Pusher = hardwareMap.get(DcMotor.class, "Pusher");

        Pusher.setDirection(DcMotorSimple.Direction.FORWARD);
        IntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        light = hardwareMap.get(RevBlinkinLedDriver.class, "light");

        motor_lf.setDirection(DcMotor.Direction.REVERSE);
        motor_lb.setDirection(DcMotor.Direction.REVERSE);
        motor_rf.setDirection(DcMotor.Direction.FORWARD);
        motor_rb.setDirection(DcMotor.Direction.FORWARD);

        motor_lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Pusher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        Shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(1);
        limelight.start();

        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        // Initialize RPM calculation
        lastShooterPosition = Shooter.getCurrentPosition();
        lastTime = System.nanoTime();

        while (opModeIsActive()) {

            LLResult llResult = limelight.getLatestResult();

            double shooterPower = noTargetPower;
            double targetRPM = 0;

            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = (y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;

            motor_lf.setPower(frontLeftPower);
            motor_lb.setPower(backLeftPower);
            motor_rf.setPower(frontRightPower);
            motor_rb.setPower(backRightPower);

            if (gamepad1.a) {
                IntakeMotor.setPower(0.8);
            } else {
                IntakeMotor.setPower(0);
            }

            if (gamepad1.b) {
                Pusher.setPower(0.9);
            } else {
                Pusher.setPower(0);
            }

            if (gamepad1.y) {
                Shooter.setPower(1);
            }
            else {
                Shooter.setPower(0);
            }

            if (gamepad1.x) {
                hood.setPosition(1);
            }
            else {
                hood.setPosition(0.5);
            }

            if (llResult != null && llResult.isValid()) {
                double tx = llResult.getTx();
                double ta = llResult.getTa();
                double error = targetTx - tx;
                double absError = Math.abs(error);
                double ty = llResult.getTy();

                if (ta > 0) {
                    // If Ta is 0.45 or less, default to max power (1.0)
                    if (ta <= 0.45) {
                        shooterPower = 1.0;
                    } else {
                        shooterPower = Range.clip(
                                maxShooterPower - (ta - minTa) *
                                        (maxShooterPower - minShooterPower) / (maxTa - minTa),
                                minShooterPower,
                                maxShooterPower
                        );
                    }
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
                telemetry.addData("Distance", distance);

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

            // Calculate RPM
            int currentPos = Shooter.getCurrentPosition();
            long now = System.nanoTime();

            int deltaPos = currentPos - lastShooterPosition;
            long deltaTime = now - lastTime;

            double ticksPerSec = (deltaPos / (deltaTime / 1e9));
            double rpm = ticksPerSec * 60.0 / 28.0;

            lastShooterPosition = currentPos;
            lastTime = now;
            boolean rpmCorrect = false;
            if (targetRPM > 0 && shooterPower > 0) {
                double rpmDifference = Math.abs(rpm - targetRPM);
                rpmCorrect = rpmDifference <= rpmTolerance;
            }

            // Set REV Blinkin LED color: Blue if correct, Red if incorrect
            if (rpmCorrect) {
                // Blue pattern
                light.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
            else {
                light.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

            }/*else {
                // Red pattern
                light.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }*/

            // Display RPM telemetry
            telemetry.addData("Shooter RPM", (int) rpm);
            if (targetRPM > 0) {
                telemetry.addData("Target RPM", (int) targetRPM);
                telemetry.addData("ΔRPM", (int) Math.abs(rpm - targetRPM));
            }
            telemetry.addData("Final Shooter Power", shooterPower);
            telemetry.update();

            sleep(20);

        }
    }
}