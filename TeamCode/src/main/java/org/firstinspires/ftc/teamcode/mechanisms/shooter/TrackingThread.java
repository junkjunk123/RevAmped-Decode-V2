package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.math.projectile.ShooterMath;
import org.firstinspires.ftc.teamcode.math.projectile.SimpleShooterMath;

@Config
public class TrackingThread {
    private final SimpleShooterMath shooterMath;
    private final Turret turret;
    private final Hood hood;
    private final Flywheel flywheel;
    private final Follower follower;
    private final boolean isTeleOp;
    public static boolean trackHood = true;
    public static boolean trackTurret = true;

    public TrackingThread(Follower follower, Turret turret, Flywheel flywheel, Hood hood, boolean isTeleOp) {
        this.hood = hood;
        this.turret = turret;
        this.flywheel = flywheel;
        shooterMath = new SimpleShooterMath(follower);
        this.isTeleOp = isTeleOp;
        this.follower = follower;
    }

    public void update() {
        if (!trackHood && !trackTurret) return;
        if (isTeleOp) follower.update();
        shooterMath.update(trackTurret, trackHood);
        hood.updateTracking(shooterMath.getHoodPos());
        turret.setTargetPosition(shooterMath.getTurretPos());
        flywheel.setVelocity(shooterMath.getFlywheelVelocity());
    }
}
