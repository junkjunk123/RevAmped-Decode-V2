package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.math.projectile.ShooterMath;

public class TrackingThread {
    private final ShooterMath shooterMath;
    private final Turret turret;
    private final Hood hood;
    private final Flywheel flywheel;
    private final Follower follower;
    private final boolean isTeleOp;
    public static boolean trackHood;
    public static boolean trackTurret;

    public TrackingThread(Follower follower, Turret turret, Flywheel flywheel, Hood hood, boolean isTeleOp) {
        this.hood = hood;
        this.turret = turret;
        this.flywheel = flywheel;
        shooterMath = new ShooterMath(follower);
        this.isTeleOp = isTeleOp;
        this.follower = follower;
    }

    public void update() {
        if (isTeleOp) follower.update();
        shooterMath.update(trackTurret, trackHood, flywheel.getTargetVelocity());
        hood.updateTracking(shooterMath.getHoodPos());
        turret.setTargetPosition(shooterMath.getTurretPos());
    }
}
