package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static double DEFAULT_PROPORTIONAL = 0.25;
    public static double MEDIUM_PROPORTIONIAL = 0.2;
    public static double CONSERVATIVE_PROPORTIONAL = 0.15;
    public static double SAFE_PROPORTIONAL = 0.1;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.4)
            .headingPIDFCoefficients(new PIDFCoefficients(1.04, 0, 0, 0.01))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2.33, 0, 0.13, 0.0005))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(
                    DEFAULT_PROPORTIONAL,
                    0.0838,
                    0.00108
            ))
            .centripetalScaling(0.00055);

    public static FollowerConstants teleopFollowerConstants = new FollowerConstants()
            .mass(12.4)
            .headingPIDFCoefficients(new PIDFCoefficients(10, 0, 0, 0.01))
            .translationalPIDFCoefficients(new PIDFCoefficients(6, 0, 0, 0.02))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2.33, 0, 0.13, 0.01))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0, 0.01))
            .translationalPIDFSwitch(0.5)
            .headingPIDFSwitch(Math.PI / 120)
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
            .forwardPodY(3.593)
            .strafePodX(-5.347)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

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

    public static Follower createFollowerTeleOp(HardwareMap hardwareMap) {
        return new FollowerBuilder(teleopFollowerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                //.setDrivetrain(new PedroMecanumDrive(hardwareMap, driveConstants))
                //.setLocalizer(new OctoQuadLocalizer(hardwareMap, octoQuadConstants, OctoQuadLocalizer.InitMode.INITIALIZE_OCTOQUAD))
                .pathConstraints(pathConstraints)
                .build();
    }
}