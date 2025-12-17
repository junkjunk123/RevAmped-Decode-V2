package org.firstinspires.ftc.teamcode.revamped;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.noop;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.revamped.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.intake.ColorManager;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.intake.IntakeDistance;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.intake.Popper;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.revamped.utils.PathSupplier;
import org.firstinspires.ftc.teamcode.revamped.utils.hardware.Encoder;

import java.util.List;

import dev.frozenmilk.dairy.mercurial.continuations.Continuation;

public class Robot {
    public static Robot INSTANCE;
    public final Drivetrain drivetrain;
    public final Turret turret;
    public final Flywheel flywheel;
    public final Table table;
    public final Hood hood;
    public final Popper popper;
    public final IntakeMotor intakeMotor;
    public final ColorManager intakeColor;
    public final IntakeDistance intakeDistance;
    private final List<LynxModule> hubs;

    private interface RobotState {
        // This could just be a marker interface; no instance needed here
    }

    private interface CycleState extends RobotState {
        class Intake implements CycleState {
            IntakeState INSTANCE = IntakeState.INTAKING;

            @Override
            public Continuation update() {
                return null;
            }
        }

        class DriveToShoot implements CycleState {
            DriveState INSTANCE = DriveState.PASSIVE;

            @Override
            public Continuation update() {
                return null;
            }
        }

        class Shoot implements CycleState {
            @Override
            public Continuation update() {
                return Continuation.halt();
            }
        }

        Continuation update();
    }

    public enum IntakeState implements RobotState {
        INTAKING,
        SORTING;
    }

    public enum DriveState implements RobotState {
        AUTO_TRACKING,
        PASSIVE;
    }


    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public Robot(HardwareMap hardwareMap, PathSupplier pathSupplier) {
        hubs = hardwareMap.getAll(LynxModule.class);
        setBulkReadMode(LynxModule.BulkCachingMode.MANUAL);
        drivetrain = pathSupplier != null ? new Drivetrain(hardwareMap, pathSupplier) : new Drivetrain(hardwareMap);
        turret = new Turret(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        intakeMotor = new IntakeMotor(hardwareMap);
        table = new Table(hardwareMap, Encoder.external(intakeMotor.get()));
        popper = new Popper(hardwareMap);
        hood = new Hood(hardwareMap);
        intakeColor = new ColorManager(hardwareMap);
        intakeDistance = new IntakeDistance(hardwareMap);
        INSTANCE = this;
    }

    public void init() {
        hood.rest();
        popper.neutral();
        table.reset();
    }

    public Continuation update = new Continuation() {
        @NonNull
        @Override
        public Continuation apply() {
            clearBulkCache();
            intakeColor.update();
            flywheel.update();
            turret.update();
            intakeMotor.update();
            table.update();
            popper.update();
            intakeDistance.update();
            hood.update();
            return update;
        }

        @Nullable
        @Override
        public StackTraceElement[] getStackTrace() {
            return null;
        }
    };

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
}
