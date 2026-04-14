package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Kinematics;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.utils.math.calc.Integrator;

@Disabled
@Autonomous
public class PedroDrivePChecker extends OpMode {
    public static int TRIALS = 2;
    private Follower follower;
    public static double DISTANCE = 40;
    private Path forward;
    private Path backward;
    private boolean followingForward = true;
    private final Integrator time = new Integrator();
    private final Integrator overshoot = new Integrator();
    private final ElapsedTime timer = new ElapsedTime();
    private int times = 0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        forward = new Path(new BezierLine(new Pose(), new Pose(DISTANCE, 0)));
        backward = new Path(new BezierLine(new Pose(DISTANCE, 0), new Pose()));
    }

    @Override
    public void start() {
        follower.followPath(forward);
        timer.reset();
    }

    @Override
    public void loop() {
        follower.update();

        if (!follower.isBusy() && times < TRIALS * 2) {
            if (followingForward) {
                follower.followPath(forward);
            } else {
                follower.followPath(backward);
            }

            addData();
            followingForward = !followingForward;
            timer.reset();
            times++;
        }

        telemetry.addData("time (ms)", time.getAvg());
        telemetry.addData("overshoot (in)", overshoot.getAvg());
        telemetry.update();
    }

    private void addData() {
        time.update(timer.milliseconds());
        overshoot.update(Kinematics.getStoppingDistance(follower.getVelocity().dot(follower.getClosestPointTangentVector().normalize()),
                follower.constants.forwardZeroPowerAcceleration));
    }
}
