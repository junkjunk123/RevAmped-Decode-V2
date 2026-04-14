package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Config
@TeleOp
public class HoldPoseTuner extends OpMode {
    private Follower follower;
    private double translationalGainThreshold = 4;
    private double headingGainThreshold = Math.PI / 15;
    private boolean reachedTip = false;

    public static double translationalP;
    public static double translationalD;
    public static double translationalF;
    public static double headingP;
    public static double headingD;
    public static double headingF;

    @Override
    public void init() {
        follower = Constants.createFollowerTeleOp(hardwareMap);
        BezierPoint start = new BezierPoint(new Pose());
        follower.followPath(new Path(start));
        follower.drivetrain.startTeleopDrive();
    }

    @Override
    public void loop() {
        //follower.setTranslationalPIDFCoefficients(new PIDFCoefficients(translationalP, 0, translationalD, translationalF));
        //follower.setHeadingPIDFCoefficients(new PIDFCoefficients(headingP, 0, headingD, headingF));
        follower.update();
    }
}
