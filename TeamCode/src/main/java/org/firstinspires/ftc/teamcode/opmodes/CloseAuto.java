package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Popper;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.mechanisms.vision.DecodeLimelight;
import org.firstinspires.ftc.teamcode.opmodes.paths.CloseAutoPaths;
import org.firstinspires.ftc.teamcode.utils.ArtifactColor;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RandomizationState;
import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.commands.Functional;
import org.firstinspires.ftc.teamcode.utils.commands.Lazy;
import org.firstinspires.ftc.teamcode.utils.logging.DecodeLogger;
import java.util.concurrent.atomic.AtomicBoolean;

public class CloseAuto extends OpModeCommand {
    private static final double TELEMETRY_PERIOD_MS = 100.0;
    private Robot robot;
    private ElapsedTime overallTimer;
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private String currentStep = "INIT";
    private static boolean testSlowShoot = false;
    protected boolean shouldPush = true;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, new CloseAutoPaths());
        DecodeLimelight limelight = new DecodeLimelight(hardwareMap);
        robot.tableCompartments.populate(ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE);
        overallTimer = new ElapsedTime();
        robot.turret.setTargetPosition(Turret.UNSORTED_AUTO_PRELOADS);

        schedule(
            new Infinite(() -> {
                robot.update();
                Pose pose = robot.drivetrain.follower.getPose();
                if (pose.distanceFrom(new Pose()) > 0.01) Drivetrain.startPose = robot.drivetrain.follower.getPose();
            }),
            new Sequential(
                    new WaitUntil(() -> !opModeInInit()),
                    new Instant(overallTimer::reset),
                    new Instant(() -> {
                        robot.turret.setTargetPosition(0);
                        robot.flywheel.closeAuto();
                        robot.hood.near();
                        robot.intakeMotor.intakeSlow();
                        limelight.setCurrentPipeline(DecodeLimelight.Pipeline.OBELISK);
                    }),
                    step("PRELOAD_DRIVE_AND_DETECT", new Race(
                            robot.drivetrain.followNext(d -> d.velocityCondition(4), 3000),
                            new Sequential(
                                    !testSlowShoot ? new Functional(() -> {}, limelight::update, () -> Globals.randomizationState != null) :
                                            new Instant(() -> Globals.randomizationState = RandomizationState.PPG),
                                    new Instant(() -> robot.turret.setTargetPosition(Turret.AUTO_PRELOADS)),
                                    new Infinite(() -> {})
                            )
                    )),
                    step("DETECTION_FALLBACK", new Lazy(() -> {
                        if (Globals.randomizationState != null) return Commands.NOOP;
                        return new Race(
                                new Functional(() -> {
                                }, limelight::update, () -> Globals.randomizationState != null),
                                new Wait(4000)
                        );
                    })),
                    step("FINALIZE_DETECTION", new Instant(() -> {
                        limelight.close();
                        robot.turret.setTargetPosition(Turret.AUTO_PRELOADS);
                        robot.intakeMotor.intakeSlow();
                    })),
                    step("PRELOAD_SHOOT", new Lazy(() -> {
                        if (Globals.randomizationState != null)
                            return robot.sortAndShootAuto();
                        return new Sequential(
                                robot.popper.pop(),
                                robot.autoFastShoot()
                        );
                    })),
                    step("INTAKE_0", intake(0)),
                    step("SHOOT_0", new Parallel(
                            new Sequential(
                                    new Instant(robot.intakeMotor::outtakeSlow),
                                    new Wait(200),
                                    new Instant(robot.intakeMotor::intake)
                            ),
                            shoot(0)
                    )),
                    step("INTAKE_1", intake(1)),
                    step("SHOOT_1", new Parallel(
                            new Sequential(
                                    new Instant(robot.intakeMotor::outtakeSlow),
                                    new Wait(200),
                                    new Instant(robot.intakeMotor::intake)
                            ),
                            shoot(1)
                    )),
                    step("INTAKE_2", intake(2)),
                    step("SHOOT_2", new Parallel(
                            new Sequential(
                                    new Instant(robot.intakeMotor::outtakeSlow),
                                    new Wait(200),
                                    new Instant(robot.intakeMotor::intake)
                            ),
                            shoot(2)
                    )),
                    step("PARK", park())
            )
        );
    }

    @Override
    public void execute() {
        if (telemetryTimer.milliseconds() >= TELEMETRY_PERIOD_MS) {
            telemetry.addData("autoStep", currentStep);
            telemetry.addData("pathIndex", robot.drivetrain.getPathIndex());
            telemetry.addData("pathsRemaining", robot.drivetrain.getRemainingPaths());
            telemetry.addData("limelightDetected", Globals.randomizationState != null);
            if (Globals.randomizationState != null) telemetry.addData("motif", Globals.randomizationState);
            telemetry.update();
            telemetryTimer.reset();
        }
    }

    @Override
    public void end() {
        //Drivetrain.startPose = robot.drivetrain.follower.getPose();
    }

    private ICommand intake(int iteration) {
        int turretPos = switch (iteration) {
            case 0 -> Turret.AUTO_SET_1;
            case 1 -> Turret.AUTO_SET_2;
            default -> Turret.AUTO_SET_3;
        };
        int driveTimeout = iteration == 2 ? 4500 : 3000;

        return new Sequential(
                new Parallel(
                        new Sequential(
                                new Wait(300),
                                new Instant(() -> {
                                    robot.turret.setTargetPosition(turretPos);
                                    robot.intakeMotor.intake();
                                    robot.flywheel.stop();
                                })
                        ),
                        iteration == 2 ? robot.resetTable() : resetTableBlock(),
                        iteration == 0 ? new Sequential(
                                new Wait(600),
                                robot.drivetrain.followNext(d -> d.tValueCondition(0.8) && d.velocityCondition(), driveTimeout)
                        ) : robot.drivetrain.followNext(d -> d.tValueCondition(0.8) && d.velocityCondition(), driveTimeout)
                )
        );
    }

    private ICommand resetTableBlock() {
        return new Sequential(
                //new Instant(robot.intakeMotor::stop),
                new Instant(() -> robot.table.setStateCommandless(Table.RelativeState.BALL1)),
                new Wait(650),
                robot.popper.blockFromPop()
        );
    }

    private ICommand shoot(int iteration) {
        double flywheelOffset = iteration == 2 ? 20 : 0;

        double driveTimeout = switch (iteration) {
            case 0 -> 3000;
            case 1 -> 2700;
            default -> 3500;
        };

        ArtifactColor[] intookColors = switch (iteration) {
            case 0 -> new ArtifactColor[] {ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE};
            case 1 -> new ArtifactColor[] {ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE};
            default -> new ArtifactColor[] {ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE};
        };

        AtomicBoolean isSorting = new AtomicBoolean();
        return new Sequential(
                new Instant(() -> isSorting.set(iteration != 2 && Globals.randomizationState != null)),
                new Parallel(
                        robot.drivetrain.followNext(d -> d.velocityCondition() && d.tValueCondition(0.8), driveTimeout),
                        iteration != 2 ? robot.popper.neutral() : Commands.NOOP,
                        new Conditional(
                                () -> iteration == 0,
                                Commands.NOOP,
                                new Sequential(
                                        new Wait(200),
                                        new Instant(() -> robot.intakeMotor.outtakeSlow())
                                )
                        ),
                        new Instant(() -> {
                                robot.flywheel.setVelocity(Flywheel.CLOSE_AUTO_VELOCITY + flywheelOffset);
                                robot.tableCompartments.populate(intookColors);
                        }),
                        new Sequential(
                                new Race(
                                        new WaitUntil(() -> iteration == 0),
                                        new Wait(300),
                                        new WaitUntil(() -> iteration == 2 && overallTimer.seconds() > 27)
                                ),
                                new Instant(() -> robot.intakeMotor.intake()),
                                new Race(
                                        new WaitUntil(() -> robot.drivetrain.tValueCondition(0.75)),
                                        new WaitUntil(() -> iteration == 2 && overallTimer.seconds() > 27.5
                                                    && overallTimer.seconds() < 28.7 && robot.drivetrain.distanceFromTarget() < 7)
                                ),
                                new Lazy(() -> {
                                    if (isSorting.get()) return robot.sortAuto();
                                    return Commands.NOOP;
                                }),
                                new Wait(150),
                                robot.popper.pop()
                        )
                ),
                robot.autoShoot(() -> !isSorting.get() ? 0.0 : Table.SLOW_SHOOT_DELAY)
        );
    }

    public ICommand park() {
        return new Parallel(
                new Sequential(
                        new Conditional(
                                () -> !shouldPush || overallTimer.seconds() > 29,
                                robot.drivetrain.followLast(d -> d.velocityCondition(4)),
                                robot.drivetrain.followNext(d -> d.velocityCondition(4))
                        ),
                        new Conditional(
                                () -> !shouldPush || overallTimer.seconds() > 29,
                                robot.drivetrain.followLast(d -> d.velocityCondition(4)),
                                robot.drivetrain.followNext(d -> d.velocityCondition(4))
                        )
                ),
                new Instant(() -> {
                    robot.flywheel.stop();
                    robot.intakeMotor.stop();
                }),
                new Conditional(
                        () -> robot.popper.atState(Popper.PopperState.NEUTRAL),
                        robot.table.reset(),
                        new Sequential(
                                robot.popper.neutral(),
                                robot.table.reset()
                        )
                ),
                robot.turret.resetTurret()
        );
    }

    @Override
    public void initializeLoop() {
        robot.update();
    }

    private ICommand step(String name, ICommand command) {
        return new Sequential(
                new Instant(() -> {
                    currentStep = name;
                    DecodeLogger.get().info("auto", "AUTO_STEP_START", "step", name);
                }),
                command,
                new Instant(() -> DecodeLogger.get().info("auto", "AUTO_STEP_COMPLETE",
                        "step", name,
                        "elapsedSec", overallTimer.seconds()))
        );
    }
}
