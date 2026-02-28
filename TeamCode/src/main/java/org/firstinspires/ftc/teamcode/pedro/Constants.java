package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static double DEFAULT_PROPORTIONAL = 0.25;
    public static double SAFE_PROPORTIONAL = 0.15;
    public static double FORWARD_PROPORTIONAL = 0.15;
    public static double BACKWARD_PROPORTIONAL = 0.3;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.4)
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0, 0.01))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.05, 0.0005))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(
                    DEFAULT_PROPORTIONAL,
                    0.090,
                    0.00125
            ))
            .centripetalScaling(0.00055);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("motor_lf")
            .leftRearMotorName("motor_lb")
            .rightFrontMotorName("motor_rf")
            .rightRearMotorName("motor_rb")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(true)
            .xVelocity(85.38)
            .yVelocity(70.47);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.5)
            .strafePodX(-2)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            50,
            1,
            10,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                //.setDrivetrain(new PedroMecanumDrive(hardwareMap, driveConstants))
                //.setLocalizer(new OctoQuadLocalizer(hardwareMap, octoQuadConstants, OctoQuadLocalizer.InitMode.INITIALIZE_OCTOQUAD))
                .pathConstraints(pathConstraints)
                .build();
    }
}