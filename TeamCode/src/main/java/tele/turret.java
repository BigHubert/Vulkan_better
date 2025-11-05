package org.firstinspires.ftc.teamcode.opmode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Turret", group = "TeleOp")
public class turret extends LinearOpMode {
    private static final double TARGETTX = 0.0;

    // Simple control parameters
    private static final double LARGE_DEADZONE = 4.5; // Much larger deadzone - don't move for small errors
    private static final double MAX_MOTOR_POWER = 0.25; // Reduced max power
    private static final double ROTATION_SPEED = 0.15; // Base rotation speed
    private static final double MIN_MOTOR_POWER = 0.08; // Minimum power to move motor

    // Shooter power constants - INVERTED for distance-based power
    private static final double MIN_SHOOTER_POWER = 0.4; // Higher minimum power for close targets
    private static final double MAX_SHOOTER_POWER = 0.9; // Maximum power for far targets
    private static final double MIN_TARGET_AREA = 0.1;   // Smallest expected target area (far)
    private static final double MAX_TARGET_AREA = 5.0;   // Largest expected target area (close)

    // Additional power when target is found and centered
    private static final double TARGET_FOUND_BONUS = 0.40; // Extra power when target is centered
    private static final double NO_TARGET_POWER = 0.1; // Very low power when no target

    double frontLeftPower = (0.15);
    double frontRightPower = (0.15);
    double backLeftPower = (0.15);
    double backRightPower = (0.15);

    private DcMotor rotationMotor;
    private Limelight3A limelight;
    private DcMotor Shooter, Shooter2;
    private DcMotor motor_rf, motor_lf, motor_rb, motor_lb, IntakeMotor;
    private Servo Pusher;
    private double lastMotorPower = 0.0;

    @Override
    public void runOpMode() {


        // Map hardware
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor_lf = hardwareMap.get(DcMotor.class, "motor_lf");
        motor_rf = hardwareMap.get(DcMotor.class, "motor_rf");
        motor_lb = hardwareMap.get(DcMotor.class, "motor_lb");
        motor_rb = hardwareMap.get(DcMotor.class, "motor_rb");

        IntakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        Pusher = hardwareMap.get(Servo.class, "Pusher");
        Pusher.setDirection(Servo.Direction.REVERSE);
        IntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_lf.setDirection(DcMotor.Direction.FORWARD);
        motor_lb.setDirection(DcMotor.Direction.REVERSE);
        motor_rf.setDirection(DcMotor.Direction.FORWARD);
        motor_rb.setDirection(DcMotor.Direction.FORWARD);
        motor_lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        Shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Shooter2 = hardwareMap.get(DcMotor.class, "Shooter2");
        Shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        Shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLResult llResult = limelight.getLatestResult();
            double shooterPower = NO_TARGET_POWER; // Start with low power
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
                IntakeMotor.setPower(0.8);
            } else {
                IntakeMotor.setPower(0);
            }

            if (gamepad1.b) {
                Pusher.setPosition(1);
            } else if (gamepad1.x) {
                Pusher.setPosition(0.0);
            } else {
                Pusher.setPosition(0.5);
            }

            if (llResult != null && llResult.isValid()) {
                double tx = llResult.getTx();
                double ta = llResult.getTa();
                double error = TARGETTX - tx;
                double absError = Math.abs(error);

                // INVERTED shooter power calculation: smaller ta (farther) = higher power
                if (ta > 0) {
                    // Inverted calculation: MAX_SHOOTER_POWER for small ta, MIN_SHOOTER_POWER for large ta
                    shooterPower = Range.clip(
                            MAX_SHOOTER_POWER - (ta - MIN_TARGET_AREA) *
                                    (MAX_SHOOTER_POWER - MIN_SHOOTER_POWER) / (MAX_TARGET_AREA - MIN_TARGET_AREA),
                            MIN_SHOOTER_POWER,
                            MAX_SHOOTER_POWER
                    );
                }

                telemetry.addData("Tx", tx);
                telemetry.addData("Ty", llResult.getTy());
                telemetry.addData("Ta", ta);
                telemetry.addData("Error", error);
                telemetry.addData("Base ShooterPower", shooterPower);

                // Simple proportional control with large deadzone
                if (absError > LARGE_DEADZONE) {
                    // Calculate motor power directly from error (no PID)
                    double motorPower = error * ROTATION_SPEED;

                    // Apply power limits
                    motorPower = Range.clip(motorPower, -MAX_MOTOR_POWER, MAX_MOTOR_POWER);

                    // Apply minimum power threshold
                    if (Math.abs(motorPower) > 0 && Math.abs(motorPower) < MIN_MOTOR_POWER) {
                        motorPower = Math.signum(motorPower) * MIN_MOTOR_POWER;
                    }

                    rotationMotor.setPower(motorPower);
                    lastMotorPower = motorPower;

                    telemetry.addData("Status", "Tracking");
                    telemetry.addData("Motor Power", motorPower);
                } else {
                    // Target is centered - increase shooter power!
                    shooterPower += TARGET_FOUND_BONUS;
                    shooterPower = Range.clip(shooterPower, 0, 1.0); // Cap at 100%

                    rotationMotor.setPower(0);
                    lastMotorPower = 0;
                    telemetry.addData("Status", "Centered - MAX POWER!");
                    telemetry.addData("Target Found Bonus", TARGET_FOUND_BONUS);
                }
            } else {
                // No target detected - use very low power
                shooterPower = NO_TARGET_POWER;
                rotationMotor.setPower(0);
                lastMotorPower = 0;
                telemetry.addData("Status", "No Data");
            }

            Shooter.setPower(shooterPower);
            Shooter2.setPower(shooterPower);

            telemetry.addData("Final Shooter Power", shooterPower);
            telemetry.update();



            sleep(20);
        }
    }
}