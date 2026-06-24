package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Conditional;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.system.CloseableOnFinalize;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler.CycleState;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler.DriveMessage;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler.Message;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeArtifactDetector;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeDistanceSensors;
import org.firstinspires.ftc.teamcode.mechanisms.lift.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.FeederWheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurretMTI;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurretState;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ShooterGate;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.TrackingThread;
import org.firstinspires.ftc.teamcode.mechanisms.vision.DecodeBlobCamera;
import org.firstinspires.ftc.teamcode.mechanisms.vision.DecodeLimelight;
import org.firstinspires.ftc.teamcode.opmodes.teleop.MTITele;
import org.firstinspires.ftc.teamcode.pedro.PathSupplier;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Lazy;
import org.firstinspires.ftc.teamcode.utils.hardware.HwVoltageSensor;
import org.firstinspires.ftc.teamcode.utils.math.projectile.SimpleShooterMath;

import java.util.List;
import java.util.concurrent.locks.Condition;

public class Robot {
    public static Robot INSTANCE;
    public final Drivetrain drivetrain;
    public final ServoTurretMTI turret;
    public final Flywheel flywheel;
    public final Hood hood;
    public final ShooterGate gate;
    public final Intake intake;
    public final FeederWheel feederWheel;
    public final HwVoltageSensor voltageSensor;
    private final List<LynxModule> hubs;
    private final HardwareMap hardwareMap;
    private CycleState robotState = CycleState.INTAKE;
    public static int SHOOT_TIME;
    public static int SHOOT_TIME_FAR;
    public static int CLEANUP_CLOSE_WAIT;
    public static int FAR_SHOOT_THRESHOLD_Y;
    public static int FAST_HOOD_COMP_THRESHOLD_VEL;
    public static boolean shootingFar;
    public static boolean useHoodComp = true;
    public static double flywheelFineTune;
    public static double hoodFineTune;

    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public Robot(HardwareMap hardwareMap, PathSupplier pathSupplier) {
        this.hardwareMap = hardwareMap;
        hubs = hardwareMap.getAll(LynxModule.class);
        setBulkReadMode(LynxModule.BulkCachingMode.MANUAL);
        voltageSensor = new HwVoltageSensor(hardwareMap);
        drivetrain = pathSupplier != null ? new Drivetrain(hardwareMap, pathSupplier) : new Drivetrain(hardwareMap);
        Globals.isTeleOp = pathSupplier == null;
        turret = new ServoTurretMTI(hardwareMap);
        flywheel = new Flywheel(hardwareMap, voltageSensor);
        intake = new Intake(hardwareMap);
        hood = new Hood(hardwareMap, voltageSensor);
        gate = new ShooterGate(hardwareMap);
        feederWheel = new FeederWheel(hardwareMap);
        shootingFar = false;
        INSTANCE = this;
        RobotStateHandler.CycleState.DRIVE_TO_SHOOT.init(drivetrain.follower, turret, hood, flywheel, Globals.isTeleOp);
        if (!Globals.isTeleOp) initialize();
    }

    public void initialize() {
        turret.move(ServoTurretState.PresetState.REST);
        hood.rest();
        gate.init();
    }

    public void update() {
        clearBulkCache();
        voltageSensor.update();
        drivetrain.update();
        flywheel.update();
        feederWheel.update();
        turret.update();
        hood.update();
        robotState.update();
        gate.update();
        intake.update();
    }

    public void setBulkReadMode(LynxModule.BulkCachingMode mode) {
        for (LynxModule module : hubs) {
            module.setBulkCachingMode(mode);
        }
    }

    public void clearBulkCache() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    public boolean isTeleop() {
        return Globals.isTeleOp;
    }

    public CycleState getRobotState() {
        return robotState;
    }

    public void setRobotState(Message message) {
        if (message instanceof DriveMessage driveState)
            CycleState.DRIVE_TO_SHOOT.INSTANCE = driveState.driveState();
        robotState = message.cycleState();
    }

//  Thilan wants hold for shooting??
//    public ICommand shoot() {
//        return new Sequential(
//                gate.open(),
//                intake()
//        );
//    }
//
//    public ICommand stopShoot(){
//        return new Sequential(
//                gate.close()
//        )
//    }

    public ICommand autoShoot(){
        return new Parallel(
            hoodComp(),
            new Sequential(
                new Instant(intake::stopSensors),
                new Instant(this::transferShoot),
                new Wait(SHOOT_TIME)
            )
        );
    }

    public ICommand autoShootFar(){
        return new Parallel(
            farHoodComp(),
            new Sequential(
                new Instant(intake::stopSensors),
                new Instant(this::transferShootFar),
                new Wait(SHOOT_TIME_FAR)
            )
        );
    }

    public ICommand resetShooter() {
        //hood.rest();
        return new Parallel(
            new Instant(() -> SimpleShooterMath.hoodCompOffset = 0),
            //new Instant(flywheel::unpower),
            gate.close()
        );
    }

    public ICommand resetAfterShooting() {
        return new Parallel(
            new Instant(drivetrain::stopHoldPose),
            resetShooter()
        );
    }

    public void intake() {
        intake.intake();
        intake.startSensors();
        feederWheel.intake();
    }

    public void transferShootFar(){
        intake.shootFar();
        feederWheel.shootFar();
    }

    public void transferShoot(){
        intake.shoot();
        feederWheel.shoot();
    }

    public void intake(boolean feederSlow){
        if (feederSlow){
            intake.intake();
            intake.startSensors();
            feederWheel.intakeSlow();
        } else {
            intake();
        }
    }

    public void outtake(){
        intake.outtake();
        intake.stopSensors();
    }

    public ICommand transfer(){
        return new Sequential(
            new Instant(this::stopIntake),
            new Wait(100),
            gate.open()
        );
    }

    public ICommand stopCleanup(){
        return new Sequential(
            new Instant(this::stopIntake),
            new Wait(CLEANUP_CLOSE_WAIT),
            resetAfterShooting()
        );
    }

    public ICommand reverseTransfer(){
        return new Sequential(
                new Instant(() ->{
                    outtake();
                    flywheel.outtake();
                }),
                new Wait(500),
                gate.close()
        );
    }

    public void stopFeeder(){
        feederWheel.stop();
    }

    public void stopIntake(){
//        intake.stopIntake();
        intake.idle();
        feederWheel.stop();
        intake.stopSensors();
    }

    public void shootNear() {
        shootingFar = false;
        hood.near();
        flywheel.near();
    }

    public void shootCorner() {
        shootingFar = false;
        hood.corner();
        flywheel.corner();
    }

    public void shootFar() {
        shootingFar = true;
        flywheel.far();
        hood.far();
    }

    public void shootMedium() {
        shootingFar = false;
        hood.medium();
        flywheel.medium();
    }

    public ICommand farHoodComp() {
        return new Conditional(
            () -> useHoodComp && Hood.HOOD_FAR_COMP != 0,
            new Sequential(
                new Wait(Hood.HOOD_COMP_DELAY),
                new Instant(hood::farHoodComp)
            ),
            Commands.NOOP
        );
    }

    public ICommand hoodComp() {
        return new Lazy(() -> {
           if (useHoodComp && Hood.HOOD_COMP != 0) {
               if (TrackingThread.trackHood) {
                   return new Sequential(
                           new Wait(Hood.HOOD_COMP_DELAY),
                           new Instant(hood::hoodComp)
                   );
               } else return new Instant(() -> {
                   double pos = hood.getPosition();
                   pos = Range.clip(pos + Hood.HOOD_COMP, 0, 1);
                   hood.setPosition(pos);
               });
           }

           return Commands.NOOP;
        });
    }
}
