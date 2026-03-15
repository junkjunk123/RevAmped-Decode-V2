package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;

import org.firstinspires.ftc.teamcode.math.projectile.SimpleShooterMath;

@Config
public class TrackingThread {
    public final SimpleShooterMath shooterMath;
    private final ServoTurret turret;
    private final Hood hood;
    private final Flywheel flywheel;
    private final Localizer pinpoint;
    private final boolean isTeleOp;
    public static boolean trackHood = false;
    public static boolean trackTurret = false;

    public TrackingThread(Follower pinpoint, ServoTurret turret, Flywheel flywheel, Hood hood, boolean isTeleOp) {
        this.hood = hood;
        this.turret = turret;
        this.flywheel = flywheel;
        this.isTeleOp = isTeleOp;
        this.pinpoint = pinpoint.getPoseTracker().getLocalizer();
        shooterMath = new SimpleShooterMath(this.pinpoint);
    }

    public void update() {
        if (!trackHood && !trackTurret) return;
        if (isTeleOp) pinpoint.update();
        shooterMath.update(trackTurret, trackHood);
        if (trackHood) hood.updateTracking(shooterMath.getHoodPos());
        if (trackTurret) turret.move(new ServoTurretState.AutoTrack(shooterMath.getTurretPos()));
        if (trackHood) flywheel.setVelocity(shooterMath.getFlywheelVelocity());
    }
}
