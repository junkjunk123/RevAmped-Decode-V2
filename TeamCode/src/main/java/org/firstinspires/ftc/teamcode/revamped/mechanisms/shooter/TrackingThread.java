package org.firstinspires.ftc.teamcode.revamped.mechanisms.shooter;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.revamped.math.calc.Angle;
import org.firstinspires.ftc.teamcode.revamped.math.projectile.ShooterMath;
import org.firstinspires.ftc.teamcode.revamped.utils.Continuations.StackTraceHelper;

import dev.frozenmilk.dairy.mercurial.continuations.Continuation;

public class TrackingThread implements Continuation {
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

    @Nullable
    @Override
    public StackTraceElement[] getStackTrace() {
        return new StackTraceElement[] {StackTraceHelper.ofClass(getClass())};
    }

    @NonNull
    @Override
    public Continuation apply() {
        if (isTeleOp) follower.update();
        shooterMath.update(trackTurret, trackHood, flywheel.getTargetVelocity());
        hood.setPosition(shooterMath.getHoodPos());
        turret.setTargetPosition(shooterMath.getTurretPos());
        return this;
    }
}
