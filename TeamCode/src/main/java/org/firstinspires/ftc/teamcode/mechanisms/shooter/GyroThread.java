package org.firstinspires.ftc.teamcode.mechanisms.shooter;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.math.projectile.FarTrackingMath;
import org.firstinspires.ftc.teamcode.utils.math.projectile.TrackState;

public class GyroThread {
    public final Localizer pinpoint;
    public final FarTrackingMath tracker;
    private final ServoTurret turret;
    private final Hood hood;
    private final Flywheel flywheel;
    private static GyroThread INSTANCE;

    public static boolean trackTurret;
    public static boolean trackTraj;

    private boolean isFar;

    public GyroThread(Follower pinpoint, ServoTurret turret, Flywheel flywheel, Hood hood) {
        this.pinpoint = pinpoint.getPoseTracker().getLocalizer();
        this.turret = turret;
        this.hood = hood;
        this.flywheel = flywheel;
        tracker = new FarTrackingMath(this.pinpoint);
        INSTANCE = this;
    }

    public GyroThread(Robot robot) {
        this(robot.drivetrain.follower, robot.turret, robot.flywheel, robot.hood);
    }

    public void setState(TrackState state) {
        isFar = state.isFar();
        tracker.setState(state).accept(flywheel, hood);
    }

    public static void offer(TrackState state) {
        if (INSTANCE != null) INSTANCE.setState(state);
    }

    public void update() {
        tracker.update();
    }

    public boolean isFar() {
        return isFar;
    }
}
