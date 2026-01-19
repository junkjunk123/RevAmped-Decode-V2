package org.firstinspires.ftc.teamcode.pedro;

import static org.firstinspires.ftc.teamcode.pedro.HeadingTuner.d;
import static org.firstinspires.ftc.teamcode.pedro.HeadingTuner.f;
import static org.firstinspires.ftc.teamcode.pedro.HeadingTuner.i;
import static org.firstinspires.ftc.teamcode.pedro.HeadingTuner.p;
import static org.firstinspires.ftc.teamcode.utils.Globals.forwardPodY;
import static org.firstinspires.ftc.teamcode.utils.Globals.strafePodX;

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

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14)
            .forwardZeroPowerAcceleration(-39.46)
            .lateralZeroPowerAcceleration(-78.17)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.03, 0, 0, 0.015))
            .translationalPIDFSwitch(4.0)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.08, 0, 0.005, 0.0006))
            .headingPIDFCoefficients(new PIDFCoefficients(p, i, d, f))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.1, 0.0005))
            .drivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.03, 0, 0.0004, 0.6,0.015)
            )
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.005,
                    0,
                    0,
                    0.6,
                    0.015))
            .drivePIDFSwitch(5)
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
            .xVelocity(76.41)
            .yVelocity(60.04);

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

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}