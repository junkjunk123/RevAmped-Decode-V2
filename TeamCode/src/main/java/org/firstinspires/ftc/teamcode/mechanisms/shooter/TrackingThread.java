package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.vision.DecodeLimelight;
import org.firstinspires.ftc.teamcode.utils.math.calc.Vector2D;
import org.firstinspires.ftc.teamcode.utils.math.projectile.FarTrackingMath;
import org.firstinspires.ftc.teamcode.utils.math.projectile.SimpleShooterMath;

@Config
public class TrackingThread {
    public final SimpleShooterMath shooterMath;
    private final ServoTurretMTI turret;
    private final Hood hood;
    private final Flywheel flywheel;
    private final Localizer pinpoint;
    public static boolean trackHood = false;
    public static boolean trackTurret = false;
    public static TrackingThread INSTANCE;
    public static boolean far = true;

    public TrackingThread(Follower pinpoint, ServoTurretMTI turret, Flywheel flywheel, Hood hood) {
        this.hood = hood;
        this.turret = turret;
        this.flywheel = flywheel;
        this.pinpoint = pinpoint.getPoseTracker().getLocalizer();
        shooterMath = new SimpleShooterMath(this.pinpoint);
        INSTANCE = this;
    }

    public TrackingThread(Robot robot) {
        this(robot.drivetrain.follower, robot.turret, robot.flywheel, robot.hood);
    }

    public void update() {
        if (!trackHood && !trackTurret) return;
        if (!far) {
            shooterMath.update(trackTurret, trackHood);
            if (trackHood) hood.updateTracking(shooterMath.getHoodPos());
            if (trackTurret)
                turret.move(new ServoTurretState.AutoTrack(shooterMath.getTurretPos()));
            if (trackHood && Robot.INSTANCE.getRobotState().equals(RobotStateHandler.CycleState.DRIVE_TO_SHOOT))
                flywheel.setVelocity(shooterMath.getFlywheelVelocity());
        }
    }

    public void addLimelightMeasurement(Vector offset) {
        Vector targ = DecodeLimelight.APRILTAG_POSE.getPose().getAsVector();
        double bearing = offset.getTheta();
        double robotHeading = pinpoint.getPose().getHeading();
        double viewRad = ServoTurret.ticksToRad(turret.getPosition()) + robotHeading + Math.PI;
        double alpha = viewRad - bearing;
        Vector tagOffset = new Vector(offset.getXComponent(), alpha);
        Vector camPos = targ.minus(tagOffset);
        Vector camOffset = new Vector(DecodeLimelight.CENTER_OFFSET, viewRad);
        Vector turretPos = camPos.minus(camOffset);
        Vector turretOffset;
        if (Turret.TURRET_OFFSET >= 0) turretOffset = new Vector2D(Turret.TURRET_OFFSET, robotHeading);
        else turretOffset = new Vector2D(Turret.TURRET_OFFSET, Math.PI + robotHeading);
        Vector robotPos = turretPos.minus(turretOffset).minus(DecodeLimelight.TAG_OFFSETS);
        pinpoint.setX(robotPos.getXComponent());
        pinpoint.setY(robotPos.getYComponent());
    }
}
