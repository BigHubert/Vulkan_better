package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.96)
            .forwardZeroPowerAcceleration(-37.29650387007365)
            .lateralZeroPowerAcceleration(-65.18653680488649)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    -0.079,
                    0,
                    -0.0012,
                    -0.021
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.78,
                    0,
                    0.005,
                    0.014
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    -0.4,
                    0,
                    -0.00001,
                    -0.6,
                    -0.14
            ))
            .centripetalScaling(0.005)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("motor_lf")
            .leftRearMotorName("motor_lb")
            .rightFrontMotorName("motor_rf")
            .rightRearMotorName("motor_rb")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(45.40695070281742)
            .yVelocity(63.74073707400345);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5.65)
            .strafePodX(-7.08)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    /**
     * These are the PathConstraints in order:
     * tValueConstraint, velocityConstraint, translationalConstraint, headingConstraint, timeoutConstraint,
     * brakingStrength, BEZIER_CURVE_SEARCH_LIMIT, brakingStart
     * <p>
     * The BEZIER_CURVE_SEARCH_LIMIT should typically be left at 10 and shouldn't be changed.
     */

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            0.1,
            0.1,
            0.1,
            100,
            1,
            10,
            1
    );

    //Add custom localizers or drivetrains here
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
