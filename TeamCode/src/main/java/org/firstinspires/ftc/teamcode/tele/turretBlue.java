package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
    private static final double rotationSpeed = 0.25;
    private static final double minMotorPower = 0.08;
    private static final double minShooterPower = 0.4;
    private static final double maxShooterPower = 1;
    private static final double minTa = 0.1;
    private static final double maxTa = 5.0;

    // Additional power when target is found and centered
    private static final double foundBonus = 0.05;
    private static final double noTargetPower = 0.0;
    private static final double llhieght = 0.25; // meters
    private static final double Theight = 1.5;   // meters
    private static final double llangle = 20.0;
    private double lastHoodPos = 0.5;   // any safe default
    private static final double hooddeadzone = 0.01; // Reduced deadzone for more responsive movement
    private static final double farDistanceThreshold = 4.0; // Distance threshold for max power (adjust as needed)
    private static final double targetLossTimeout = 500.0; // 0.5 seconds in milliseconds

    // Gamepad2 manual shooter power settings
    private static final double manualPowerLow = 0.6;
    private static final double manualPowerHigh = 0.8;

    // Far distance speed reduction constants
    private static final double farTaThreshold = 0.5; // Ta threshold for far distances
    private static final double farDistanceSpeedMultiplier = 0.10; // Speed multiplier when Ta < 0.5 (10% of normal speed, reduced from 0.15)

    // Anti-bounce constants for small errors
    private static final double rampZoneStart = 12.0; // Start reducing speed when error is below this (degrees, increased from 10.0)
    private static final double minSpeedMultiplier = 0.03; // Minimum speed multiplier when very close to target (reduced from 0.05 for more aggressive stopping)
    private static final double closeErrorThreshold = 8.0; // Error threshold where we start reducing minMotorPower (increased from 7.0)

    // Aggressive stopping constants to prevent overshoot
    private static final double brakingZone = 7.0; // Start aggressive braking when error is below this (degrees, increased from 6.0)
    private static final double brakingPower = 0.15; // Reverse power to brake when approaching target
    private static final double stopZone = 6.0; // Zone where we stop completely to prevent overshoot (increased from 5.0)

    private boolean lastHadTarget = false;
    double frontLeftPower = (0.15);
    double frontRightPower = (0.15);
    double backLeftPower = (0.15);
    double backRightPower = (0.15);
    private double distance;

    private DcMotor rotationMotor;
    private Limelight3A limelight;
    private DcMotor Shooter;
    private DcMotor motor_rf, motor_lf, motor_rb, motor_lb, IntakeMotor, Pusher;
    private Servo hood;
    private Servo light;

    private double lastMotorPower = 0.0;

    // Shooter power persistence variables
    private double lastValidShooterPower = 0.0;
    private long targetLostTime = 0;

    /**
     * Calculate hood position based on target area (Ta)
     * Higher Ta (closer target) = lower position value
     * Lower Ta (farther target) = higher position value
     */
    private double getHoodPositionFromTa(double ta) {
        // Clamp Ta to valid range
        ta = Range.clip(ta, minTa, maxTa);

        // Normalize Ta to 0-1 range (NOT inverted: high Ta -> high normalized value)
        double normalizedTa = ((ta - minTa) / (maxTa - minTa));

        // Map to servo position range (adjust these values based on your servo limits)
        // Position range: 0.3 (far) to 0.7 (close) - INVERTED
        double minPos = 0.3;
        double maxPos = 0.7;
        double pos = minPos + normalizedTa * (maxPos - minPos);

        return Range.clip(pos, 0.0, 1.0);
    }

    private double getShooterPowerFromTa(double ta) {
        // Piecewise linear interpolation based on Ta values

        if (ta >= 3.5) {
            return 0.5;
        } else if (ta >= 2.1) {
            // Interpolate between (3.5, 0.5) and (2.1, 0.6)
            double t = (ta - 2.1) / (3.5 - 2.1);
            return 0.6 + t * (0.5 - 0.6);
        } else if (ta >= 1.4) {
            // Interpolate between (2.1, 0.6) and (1.4, 0.65)
            double t = (ta - 1.4) / (2.1 - 1.4);
            return 0.66 + t * (0.6 - 0.65);
        } else if (ta >= 0.8) {
            // Interpolate between (1.4, 0.65) and (0.8, 0.7)
            double t = (ta - 0.8) / (1.4 - 0.8);
            return 0.699 + t * (0.65 - 0.7);
        } else if (ta >= 0.35) {
            // Interpolate between (0.8, 0.7) and (0.35, 0.8)
            double t = (ta - 0.35) / (0.8 - 0.35);
            return 0.78 + t * (0.7 - 0.8);
        } else if (ta >= 0.32) {
            // Interpolate between (0.35, 0.8) and (0.32, 0.82)
            double t = (ta - 0.32) / (0.35 - 0.32);
            return 0.78 + t * (0.8 - 0.82);
        } else {
            // For Ta < 0.32, use max power
            return 0.8;
        }
    }

    /**
     * Calculate speed multiplier based on Ta (target area)
     * When Ta < 0.5 (far distances), reduce speed significantly to prevent bouncing
     */
    private double getSpeedMultiplierFromTa(double ta) {
        if (ta < farTaThreshold) {
            return farDistanceSpeedMultiplier;
        } else {
            return 1.0; // Normal speed for closer targets
        }
    }

    /**
     * Calculate speed multiplier based on error magnitude to prevent bouncing
     * Smaller errors = slower speed for more aggressive stopping
     */
    private double getSpeedMultiplierFromError(double absError) {
        if (absError <= deadzone) {
            // Already in deadzone, should be stopped
            return 0.0;
        } else if (absError <= stopZone) {
            // In stop zone - use minimum speed
            return minSpeedMultiplier;
        } else if (absError <= brakingZone) {
            // In braking zone - rapidly reduce speed
            // Quadratic curve for more aggressive reduction
            double t = (absError - stopZone) / (brakingZone - stopZone);
            double speed = minSpeedMultiplier + (0.3 - minSpeedMultiplier) * (1.0 - t * t);
            return speed;
        } else if (absError <= rampZoneStart) {
            // In ramp zone - gradually reduce speed as we approach braking zone
            // Quadratic interpolation for faster reduction
            double t = (absError - brakingZone) / (rampZoneStart - brakingZone);
            double speed = 0.3 + (1.0 - 0.3) * (1.0 - t * t);
            return speed;
        } else {
            // Far from target, use full speed
            return 1.0;
        }
    }

    /**
     * Apply aggressive braking when approaching target to prevent overshoot
     * Returns the motor power with braking applied if we're in the braking zone
     */
    private double applyAggressiveBraking(double motorPower, double error, double absError) {
        // If we're in the stop zone, stop immediately
        if (absError <= stopZone) {
            return 0.0;
        }

        // If we're in the braking zone and moving toward the target, apply reverse braking
        if (absError <= brakingZone) {
            // Check if we're moving toward the target (error and power have opposite signs means moving toward)
            // If error is positive and power is negative, we're moving left toward center
            // If error is negative and power is positive, we're moving right toward center
            boolean movingTowardTarget = (error > 0 && motorPower < 0) || (error < 0 && motorPower > 0);

            if (movingTowardTarget) {
                // Apply reverse braking proportional to how close we are
                double brakeFactor = (brakingZone - absError) / brakingZone; // 0 at edge, 1 at stopZone
                double brakePower = -Math.signum(error) * brakingPower * brakeFactor;

                // Reduce forward power and add braking
                motorPower = motorPower * (1.0 - brakeFactor * 0.5) + brakePower;
            }
        }

        return motorPower;
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
        // Initialize hood to safe position
        hood.setPosition(lastHoodPos);

        motor_lf = hardwareMap.get(DcMotor.class, "motor_lf");
        motor_rf = hardwareMap.get(DcMotor.class, "motor_rf");
        motor_lb = hardwareMap.get(DcMotor.class, "motor_lb");
        motor_rb = hardwareMap.get(DcMotor.class, "motor_rb");

        IntakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        Pusher = hardwareMap.get(DcMotor.class, "Pusher");

        Pusher.setDirection(DcMotorSimple.Direction.FORWARD);
        IntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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

        light = hardwareMap.get(Servo.class, "light");

        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            LLResult llResult = limelight.getLatestResult();

            // Initialize shooter power
            double shooterPower = noTargetPower;
            boolean gamepad2Active = false;
            double manualShooterPower = 0.0;

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
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
                IntakeMotor.setPower(1);
            } else {
                IntakeMotor.setPower(0);
            }

            if (gamepad1.b) {
                Pusher.setPower(1);
            } else {
                Pusher.setPower(0);
            }

            // ===== GAMEPAD2 MANUAL SHOOTER CONTROL (OVERRIDES LIMELIGHT WHEN PRESSED) =====
            if (gamepad2.a) {
                // Low power setting (0.6)
                manualShooterPower = manualPowerLow;
                gamepad2Active = true;
                lastValidShooterPower = manualShooterPower;
            } else if (gamepad2.b) {
                // High power setting (0.8)
                manualShooterPower = manualPowerHigh;
                gamepad2Active = true;
                lastValidShooterPower = manualShooterPower;
            } else if (gamepad2.x) {
                // Stop shooter
                manualShooterPower = noTargetPower;
                gamepad2Active = true;
                lastValidShooterPower = 0.0;
            }

            // Limelight controls rotation, hood, and shooter power (when gamepad2 not active)
            if (llResult != null && llResult.isValid()) {
                double tx = llResult.getTx();
                double ta = llResult.getTa();
                double error = targetTx - tx;
                double absError = Math.abs(error);
                double ty = llResult.getTy();
                distance = getdistancefromtarget(llResult.getTa());

                // Calculate shooter power from limelight ONLY if gamepad2 is not being used
                if (!gamepad2Active && ta > 0) {
                    shooterPower = getShooterPowerFromTa(ta);
                    shooterPower = Range.clip(shooterPower, 0.0, 1.0);
                    lastValidShooterPower = shooterPower;
                }

                // Reset target lost time
                targetLostTime = 0;

                // Calculate hood position directly from Ta (target area)
                double hoodPos = getHoodPositionFromTa(ta);

                // Always update hood position if it's different
                if (Math.abs(hoodPos - lastHoodPos) > hooddeadzone) {
                    hood.setPosition(hoodPos);
                    lastHoodPos = hoodPos;
                }

                lastHadTarget = true;
                telemetry.addData("Hood", "Tracking target");
                telemetry.addData("Distance", distance);
                telemetry.addData("Ta", "%.3f", ta);
                telemetry.addData("Hood Pos", "%.3f", hoodPos);

                if (absError > deadzone) {
                    // Calculate base motor power
                    double motorPower = error * rotationSpeed;

                    // Apply speed multiplier based on Ta (reduce speed for far distances)
                    double taMultiplier = getSpeedMultiplierFromTa(ta);

                    // Apply speed multiplier based on error (reduce speed when close to target)
                    double errorMultiplier = getSpeedMultiplierFromError(absError);

                    // Combine both multipliers (use the smaller one for more conservative control)
                    double speedMultiplier = Math.min(taMultiplier, errorMultiplier);
                    motorPower *= speedMultiplier;

                    motorPower = Range.clip(motorPower, -maxMotorPower, maxMotorPower);

                    // Reduce or eliminate minimum motor power when very close to target
                    // This prevents the motor from forcing movement when it should be slowing down
                    double effectiveMinPower = minMotorPower;
                    if (absError < closeErrorThreshold) {
                        // Gradually reduce minimum power as we get closer
                        double t = absError / closeErrorThreshold;
                        effectiveMinPower = minMotorPower * t;
                        // Don't let it go below a very small value to prevent stalling
                        effectiveMinPower = Math.max(effectiveMinPower, 0.02);
                    }

                    if (Math.abs(motorPower) > 0 && Math.abs(motorPower) < effectiveMinPower) {
                        motorPower = Math.signum(motorPower) * effectiveMinPower;
                    }

                    // Apply aggressive braking to prevent overshoot
                    motorPower = applyAggressiveBraking(motorPower, error, absError);
                    motorPower = Range.clip(motorPower, -maxMotorPower, maxMotorPower);

                    rotationMotor.setPower(motorPower);
                    lastMotorPower = motorPower;

                    telemetry.addData("Status", "Tracking");
                    telemetry.addData("Motor Power", "%.3f", motorPower);
                    telemetry.addData("Ta Multiplier", "%.2f", taMultiplier);
                    telemetry.addData("Error Multiplier", "%.2f", errorMultiplier);
                    telemetry.addData("Final Multiplier", "%.2f", speedMultiplier);
                    telemetry.addData("Error", "%.2f", absError);
                } else {
                    // Add bonus when centered (only if using limelight power)
                    if (!gamepad2Active) {
                        shooterPower += foundBonus;
                        shooterPower = Range.clip(shooterPower, 0, 1.0);
                        lastValidShooterPower = shooterPower;
                    }

                    // Stop immediately when in deadzone - no smoothing
                    rotationMotor.setPower(0.0);
                    lastMotorPower = 0.0;

                    telemetry.addData("Status", "Centered");
                    telemetry.addData("Motor Power", "0.000");
                }

            } else {
                // Target lost - maintain power if gamepad2 not active
                if (!gamepad2Active) {
                    long currentTime = System.currentTimeMillis();

                    if (targetLostTime == 0) {
                        targetLostTime = currentTime;
                    }

                    double timeSinceLost = currentTime - targetLostTime;

                    if (timeSinceLost < targetLossTimeout && lastValidShooterPower > 0) {
                        // Maintain previous shooter power for timeout period
                        shooterPower = lastValidShooterPower;
                    } else {
                        // Timeout expired - use no target power
                        shooterPower = noTargetPower;
                    }
                }

                rotationMotor.setPower(0);
                lastMotorPower = 0;
                telemetry.addData("Status", "No Data");
                hood.setPosition(lastHoodPos);
                telemetry.addData("Hood", "No target - holding last position");
                lastHadTarget = false;
            }

            // Apply shooter power (gamepad2 overrides, otherwise use limelight)
            if (gamepad2Active) {
                shooterPower = manualShooterPower;
            }

            Shooter.setPower(shooterPower);

            // Display telemetry
            if (gamepad2Active) {
                telemetry.addData("Mode", "MANUAL (Gamepad2)");
                telemetry.addData("Shooter Power", "%.2f", shooterPower);
            } else {
                telemetry.addData("Mode", "AUTO (Limelight)");
                telemetry.addData("Shooter Power", "%.2f", shooterPower);
            }
            telemetry.addData("Gamepad2", "A=0.6, B=0.8, X=Stop");
            telemetry.update();

            sleep(20);

        }
    }
    public double getdistancefromtarget(double ta) {
        double scale = 30899.182;
        double distance = (scale / ta);
        return distance;
    }
}

