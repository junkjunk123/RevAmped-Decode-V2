package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler.CycleState;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler.DriveMessage;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler.IntakeMessage;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler.Message;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeArtifactDetector;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeTilt;
import org.firstinspires.ftc.teamcode.mechanisms.lift.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.FeederWheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurretState;
import org.firstinspires.ftc.teamcode.mechanisms.vision.DecodeBlobCamera;
import org.firstinspires.ftc.teamcode.mechanisms.vision.DecodeLimelight;
import org.firstinspires.ftc.teamcode.pedro.PathSupplier;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.hardware.HwVoltageSensor;

import java.util.List;

public class Robot {
    public static Robot INSTANCE;
    public final Drivetrain drivetrain;
    public final ServoTurret turret;
    public final Flywheel flywheel;
    public final Hood hood;
    public final IntakeMotor intakeMotor;
    public final IntakeArtifactDetector ball1Distance;
    public final IntakeArtifactDetector ball2Distance;
    public final IntakeArtifactDetector ball3Distance;
    public final FeederWheel feederWheel;
    public final IntakeTilt intakeTilt;
    public final DecodeLimelight limelight;
    public final Lift lift;
    public final DecodeBlobCamera intakeCamera;
    public final HwVoltageSensor voltageSensor;

    private final List<LynxModule> hubs;
    private final HardwareMap hardwareMap;
    private CycleState robotState = CycleState.INTAKE;

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
        turret = new ServoTurret(hardwareMap);
        flywheel = new Flywheel(hardwareMap, voltageSensor);
        intakeMotor = new IntakeMotor(hardwareMap);
        hood = new Hood(hardwareMap, voltageSensor);
        feederWheel = new FeederWheel(hardwareMap);
        intakeTilt = new IntakeTilt(hardwareMap);
        limelight = new DecodeLimelight(hardwareMap);
        lift = new Lift(hardwareMap);
        intakeCamera = new DecodeBlobCamera(hardwareMap);
        ball1Distance = new IntakeArtifactDetector(hardwareMap,"ball1");
        ball2Distance = new IntakeArtifactDetector(hardwareMap,"ball2");
        ball3Distance = new IntakeArtifactDetector(hardwareMap,"ball3");
        INSTANCE = this;
        RobotStateHandler.CycleState.DRIVE_TO_SHOOT.init(drivetrain.follower, turret, hood, flywheel, Globals.isTeleOp);
        if (!Globals.isTeleOp) initialize();
    }

    public void initialize() {
        turret.move(ServoTurretState.PresetState.REST);
        intakeTilt.intake();
        hood.rest();
    }

    public void update() {
        clearBulkCache();
        voltageSensor.update();
        drivetrain.update();
        flywheel.update();
        feederWheel.update();
        limelight.update();
        intakeCamera.update();
        turret.update();
        intakeMotor.update();
        intakeTilt.update();
        hood.update();
        robotState.update();
        ball1Distance.update();
        ball2Distance.update();
        ball3Distance.update();
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
        else if (message instanceof IntakeMessage intakeMessage)
            CycleState.INTAKE.INSTANCE = intakeMessage.intakeState();
        else if (message instanceof CycleState.Intake)
            CycleState.INTAKE.INSTANCE = RobotStateHandler.IntakeState.INTAKING;
        robotState = message.cycleState();
    }

    public ICommand shootAll() {
        return new Sequential(
                new WaitUntil(() -> drivetrain.canShoot),
                new Instant(() -> {
                    CycleState.INTAKE.update = true;
                }),
                new Conditional(
                    () -> hood.atState(Hood.HoodState.FAR),
                    new Parallel(
                        new Sequential(
                            //half-assed hood comp
                            new Wait(50),
                            new Instant(hood::farHoodComp)
                            ),
                        intake()
                    ),
                    intake()
                )
        );
    }

    public ICommand autoFastShoot(IntakeTilt.TiltState tiltState) {
        return new Sequential(
                new Instant(() -> {
                    if (tiltState.equals(IntakeTilt.TiltState.INTAKE)) intakeTilt.intake();
                    else if (tiltState.equals(IntakeTilt.TiltState.GATE_INTAKE)) intakeTilt.gateIntake();
                }),
                shootAll()
        );
    }

    public ICommand autoFastShoot() {
        return autoFastShoot(IntakeTilt.TiltState.INTAKE);
    }

    public ICommand resetShooter() {
        return new Instant(() -> {
            flywheel.stop();
            //hood.rest();
            feederWheel.stop();
        });
    }

    public ICommand resetAfterShooting() {
        return new Parallel(
                new Instant(drivetrain::stopHoldPose),
                resetShooter()
        );
    }

    public void shootNear() {
        hood.near();
        flywheel.near();
    }

    public void shootCorner() {
        hood.corner();
        flywheel.corner();
    }

    public void shootFar() {
        flywheel.far();
        hood.far();
    }

    public void shootMedium() {
        hood.medium();
        flywheel.medium();
    }

    public void close() {
        limelight.close();
        intakeCamera.close();
    }

    public ICommand lift() {
        return new Sequential(
                new Instant(() -> {
                    limelight.close();
                    turret.deenergize();
                    intakeMotor.deenergize();
                    hood.deenergize();
                    feederWheel.deenergize();
                    intakeTilt.deenergize();
                    flywheel.deenergize();
                }),
                lift.lift()
        );
    }

    public ICommand intake() {
        return intake(0);
    }

    public ICommand intake(float tiltFinetune) {
        return new Sequential(
                new Instant(() -> intakeTilt.intake(tiltFinetune)),
                new Instant(() -> {
                    intakeMotor.intake();
                    feederWheel.intakeState();
                })
        );
    }

    public ICommand gateIntake() {
        return new Sequential(
                new Instant(intakeTilt::gateIntake),
                new Instant(() -> {
                    intakeMotor.intake();
                    feederWheel.intakeState();
                })
        );
    }
}
