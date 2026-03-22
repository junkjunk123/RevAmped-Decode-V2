package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.math.projectile.SimpleShooterMath;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler;
import org.firstinspires.ftc.teamcode.utils.Globals;

@Config
public class TrackingThread {
    public final SimpleShooterMath shooterMath;
    private final ServoTurret turret;
    private final Hood hood;
    private final Flywheel flywheel;
    private final Localizer pinpoint;
    public static boolean trackHood = false;
    public static boolean trackTurret = false;

    public TrackingThread(Follower pinpoint, ServoTurret turret, Flywheel flywheel, Hood hood) {
        this.hood = hood;
        this.turret = turret;
        this.flywheel = flywheel;
        this.pinpoint = pinpoint.getPoseTracker().getLocalizer();
        shooterMath = new SimpleShooterMath(this.pinpoint);
    }

    public void update() {
        if (!trackHood && !trackTurret) return;
        shooterMath.update(trackTurret, trackHood);
        if (trackHood) hood.updateTracking(shooterMath.getHoodPos());
        if (trackTurret) turret.move(new ServoTurretState.AutoTrack((float) shooterMath.getTurretPos()));
        if (trackHood && Robot.INSTANCE.getRobotState().equals(RobotStateHandler.CycleState.DRIVE_TO_SHOOT))
            flywheel.setVelocity(shooterMath.getFlywheelVelocity());
    }
}
