package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Disabled
@Config
@TeleOp
public class PedroPIDChecker extends OpMode {
    private Follower follower;
    public static double TIMEOUT = 2.5; //seconds
    public static double ANGULAR_VEL_CONSTRAINT = 0.007;
    private ElapsedTime timer = new ElapsedTime();
    private boolean done = false;
    private boolean translationalDone = false;
    private boolean headingDone = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
    }

    @Override
    public void start() {
        follower.holdPoint(new Pose(10, 10, Math.PI / 2));
    }

    @Override
    public void loop() {
        follower.update();
        telemetry.update();

        if (!done) {
            if (timer.seconds() > TIMEOUT) {
                done = true;
                return;
            }

            if (!translationalDone)
                if (follower.getVelocity().getMagnitude() < PathConstraints.defaultConstraints.getVelocityConstraint())
                    setTranslationalDone();
            if (!headingDone)
                if (follower.getAngularVelocity() < ANGULAR_VEL_CONSTRAINT)
                    setHeadingDone();
            return;
        }

        telemetry.addData("translationalPassed", follower.getTranslationalError().getMagnitude()
                < PathConstraints.defaultConstraints.getTranslationalConstraint());
        telemetry.addData("headingPassed", follower.getHeadingError() < PathConstraints.defaultConstraints.getHeadingConstraint());
    }

    private void setHeadingDone() {
        headingDone = true;
        if (translationalDone) done = true;
    }

    private void setTranslationalDone() {
        translationalDone = true;
        if (headingDone) done = true;
    }
}
