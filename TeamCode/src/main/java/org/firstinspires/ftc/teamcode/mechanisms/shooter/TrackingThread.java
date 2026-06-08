package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler;
import org.firstinspires.ftc.teamcode.utils.math.projectile.SimpleShooterMath;

@Config
public class TrackingThread {
    public final SimpleShooterMath shooterMath;
    private final ServoTurretMTI turret;
    private final Hood hood;
    private final Flywheel flywheel;
    private final Localizer octoquad;
    public static boolean trackHood = true;
    public static boolean trackTurret = true;
    public static TrackingThread INSTANCE;
    public static boolean far = false;
    public static float TURRET_OFFSET;
    public static boolean velocityCompensation = false;

    public TrackingThread(Follower octoquad, ServoTurretMTI turret, Flywheel flywheel, Hood hood) {
        this.hood = hood;
        this.turret = turret;
        this.flywheel = flywheel;
        this.octoquad = octoquad.getPoseTracker().getLocalizer();
        shooterMath = new SimpleShooterMath(this.octoquad);
        INSTANCE = this;
    }

    public TrackingThread(Robot robot) {
        this(robot.drivetrain.follower, robot.turret, robot.flywheel, robot.hood);
    }

    public void update() {
        if (!trackHood && !trackTurret) return;
        if (!far) {
            shooterMath.update(trackTurret, trackHood);
            if (trackHood) {
                hood.updateTracking(shooterMath.getHoodPos());

            }
            if (trackTurret)
                turret.move(new ServoTurretState.AutoTrack(shooterMath.getTurretPos()));
            if (trackHood && Robot.INSTANCE.getRobotState().equals(RobotStateHandler.CycleState.DRIVE_TO_SHOOT))
                flywheel.setVelocity(shooterMath.getFlywheelVelocity());
        }
    }

//    public void addLimelightMeasurement(Vector offset) {
//        Vector targ = DecodeLimelight.APRILTAG_POSE.getPose().getAsVector();
//        double bearing = offset.getTheta();
//        double robotHeading = octoquad.getPose().getHeading();
//        double viewRad = ServoTurret.ticksToRad(turret.getPosition()) + robotHeading + Math.PI;
//        double alpha = viewRad - bearing;
//        Vector tagOffset = new Vector(offset.getXComponent(), alpha);
//        Vector camPos = targ.minus(tagOffset);
//        Vector camOffset = new Vector(DecodeLimelight.CENTER_OFFSET, viewRad);
//        Vector turretPos = camPos.minus(camOffset);
//        Vector turretOffset;
//        if (Turret.TURRET_OFFSET >= 0) turretOffset = new Vector2D(Turret.TURRET_OFFSET, robotHeading);
//        else turretOffset = new Vector2D(Turret.TURRET_OFFSET, Math.PI + robotHeading);
//        Vector robotPos = turretPos.minus(turretOffset).minus(DecodeLimelight.TAG_OFFSETS);
//        octoquad.setX(robotPos.getXComponent());
//        octoquad.setY(robotPos.getYComponent());
//    }
}
