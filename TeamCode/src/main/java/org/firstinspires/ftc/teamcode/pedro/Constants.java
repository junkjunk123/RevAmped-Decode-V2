package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
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
import org.firstinspires.ftc.teamcode.mechanisms.PedroMecanumDrive;
import org.firstinspires.ftc.teamcode.pedro.octoquad.OctoQuadConstants;
import org.firstinspires.ftc.teamcode.pedro.octoquad.OctoQuadFWv3;
import org.firstinspires.ftc.teamcode.pedro.octoquad.OctoQuadLocalizer;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.4)
            .forwardZeroPowerAcceleration(-29.08)
            .lateralZeroPowerAcceleration(-69)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.03, 0, 0, 0.015))
            .translationalPIDFSwitch(4.0)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.08, 0, 0.005, 0.0006))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0, 0.01))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.1, 0.0005))
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
            .xVelocity(85.38)
            .yVelocity(70.47);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(2.2)
            .strafePodX(-1.15)
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

    public static OctoQuadConstants octoQuadConstants = new OctoQuadConstants()
            .name("octoquad")
            .deadwheelPortX(1)
            .deadwheelPortY(0)
            .deadwheelXDir(OctoQuadFWv3.EncoderDirection.REVERSE)
            .deadwheelYDir(OctoQuadFWv3.EncoderDirection.FORWARD)
            .deadwheelXTicksPerMM(19.89436789f)
            .deadwheelYTicksPerMM(19.89436789f)
            .tcpOffsetXMM((float) DistanceUnit.MM.fromInches(-3.46))
            .tcpOffsetYMM((float) DistanceUnit.MM.fromInches(5.175))
            .imuScalar(0.9731f);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                //.mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .setDrivetrain(new PedroMecanumDrive(hardwareMap, driveConstants))
                //.setLocalizer(new OctoQuadLocalizer(hardwareMap, octoQuadConstants, OctoQuadLocalizer.InitMode.INITIALIZE_OCTOQUAD))
                .pathConstraints(pathConstraints)
                .build();
    }
}