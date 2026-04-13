package org.firstinspires.ftc.teamcode.sim;

import android.app.Application;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.utils.math.Ellipse2D;
import org.firstinspires.ftc.teamcode.utils.math.calc.Vector2D;

public class PedroRobot {
    public final Follower follower;
    private final Ellipse2D velocityEllipse;
    private final double maxAngularVel;
    public final PedroRobotLocalizer localizer;

    public PedroRobot(Follower follower, Pose startPose, Ellipse2D velocityEllipse, double maxAngularVel) {
        this.follower = follower;
        this.velocityEllipse = velocityEllipse;
        this.maxAngularVel = maxAngularVel;
        localizer = new PedroRobotLocalizer(this);
        localizer.setStartPose(startPose);
    }

    public void update(Vector2D linearVel, double angularVel) {
        localizer.update(velocityEllipse.interpolate(linearVel), angularVel * maxAngularVel);
    }
}
