package org.firstinspires.ftc.teamcode.opmodes;
import static org.firstinspires.ftc.teamcode.opmodes.AutoMethods.resetTable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Deadline;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Popper;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.mechanisms.vision.DecodeLimelight;
import org.firstinspires.ftc.teamcode.opmodes.paths.FifteenSortedPaths;
import org.firstinspires.ftc.teamcode.utils.ArtifactColor;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RandomizationState;
import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.commands.Functional;
import org.firstinspires.ftc.teamcode.utils.commands.Lazy;

import java.util.concurrent.atomic.AtomicBoolean;

public class FifteenSorted extends OpModeCommand {
    private Robot robot;
    private final ElapsedTime overallTimer = new ElapsedTime();
    private static boolean testSlowShoot = true;
    private boolean transferredData = false;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, new FifteenSortedPaths());
        DecodeLimelight limelight = new DecodeLimelight(hardwareMap);
        robot.tableCompartments.populate(ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE);

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
                            robot.turret.setPosition(ServoTurret.FIFTEEN_OBELISK_DETECTION);
                            robot.flywheel.unsortedAuto();
                            robot.hood.unsortedAuto();
                            robot.intakeMotor.intakeSlow();
                            limelight.setCurrentPipeline(DecodeLimelight.Pipeline.OBELISK);
                        }),
                        new Race(
                                robot.drivetrain.followNext(d -> d.velocityCondition(4), 3000),
                                new Sequential(
                                        new Wait(300),
                                        !testSlowShoot ? new Functional(() -> {}, limelight::update, () -> Globals.randomizationState != null) :
                                                new Instant(() -> Globals.randomizationState = RandomizationState.GPP),
                                        new Instant(() -> robot.turret.setPosition(ServoTurret.FIFTEEN_BALL_PRELOADS)),
                                        new Infinite(() -> {})
                                )
                        ),
                        new Lazy(() -> {
                            if (Globals.randomizationState != null) return Commands.NOOP;
                            return new Race(
                                    new Functional(() -> {
                                    }, limelight::update, () -> Globals.randomizationState != null),
                                    new Wait(4000)
                            );
                        }),
                        new Instant(() -> {
                            limelight.close();
                            robot.turret.setPosition(ServoTurret.FIFTEEN_BALL_PRELOADS - 3/255f); //looks fine
                            robot.intakeMotor.intakeSlow();
                        }),
                        robot.popper.pop(),
                        new Parallel(
                                robot.autoFastShoot(),
                                new Sequential(
                                        new Wait(100),
                                        new Instant(() -> robot.turret.setPosition(robot.turret.getPosition() + 3/255f))
                                )
                        ),
                        intakePreload(robot, true),
                        shootPreload(robot, true), //uses unsorted final
                        intakeFromGate(robot, 0),
                        shootFromGate(robot, 0, true),//uses unsorted auto set 1
                        intakeForSort(robot, 0), //uses auto set 1
                        shootForSort(overallTimer, robot, 0),
                        intakeForSort(robot, 2),
                        intakeLastPreloadSet(),
                        shootLastPreloadSet(),
                        new Parallel(
                                new Instant(() -> {
                                    robot.flywheel.stop();
                                    robot.intakeMotor.stop();
                                    Globals.turretStartPos = robot.turret.getPosition();
                                    transferredData = true;
                                }),
                                new Conditional(
                                        () -> robot.popper.atState(Popper.PopperState.NEUTRAL),
                                        robot.table.reset(),
                                        new Sequential(
                                                robot.popper.neutral(),
                                                robot.table.reset()
                                        )
                                )
                        )
                )
        );
    }

    private ICommand intakeLastPreloadSet() {
        return new Sequential(
                new Parallel(
                        new Sequential(
                                new Wait(300),
                                new Instant(() -> {
                                    robot.turret.setPosition(ServoTurret.AUTO_SET_3);
                                    robot.intakeMotor.intake();
                                    robot.flywheel.stop();
                                })
                        ),
                        resetTable(robot),
                        new Instant(robot.intakeMotor::outtakeMidSlow),
                        robot.drivetrain.followNext(d -> d.tValueCondition(0.8) && d.velocityCondition(), 4500)
                )
        );
    }

    private ICommand shootLastPreloadSet() {
        ArtifactColor[] colors = new ArtifactColor[] {ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE};
        return new Sequential(
                new Parallel(
                        robot.drivetrain.followNext(d -> d.velocityCondition() && d.tValueCondition(0.8), 4500),
                        new Sequential(
                                new Wait(200),
                                new Instant(robot.intakeMotor::outtakeMidSlow),
                                new Lazy(() -> {
                                    if (Globals.randomizationState != null) return robot.sortAuto();
                                    return Commands.NOOP;
                                }),
                                robot.popper.pop()
                        ),
                        new Instant(() -> {
                            robot.flywheel.setVelocity(Flywheel.NEAR_VELOCITY-100);
                            robot.hood.setPosition(Hood.NEAR_PRESET - 25/255f);
                            robot.tableCompartments.populate(colors);
                        })
                ),
                robot.autoShoot(() -> Globals.randomizationState == null ? 0.0 : Table.SLOW_SHOOT_DELAY)
        );
    }

    public static ICommand intakeFromGate(Robot robot, int iteration) {
        return new Sequential(
                new Instant(() -> {
                    robot.flywheel.stop();
                    robot.intakeMotor.stop();
                }),
                new Parallel(
                        resetTable(robot),
                        new Sequential(
                                robot.drivetrain.followNext(d -> d.tValueCondition(0.8) && d.velocityCondition(), 3000),
                                new Instant(() -> {
                                    robot.turret.unsortedAutoSet(iteration + 1);
                                    robot.flywheel.setVelocity(Flywheel.UNSORTED_AUTO_VELOCITY-20);
                                })
                        ),
                        new Sequential(
                                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.75)),
                                new Instant(() -> {
                                    robot.intakeMotor.intakeGate();
                                    robot.turret.setPosition(robot.turret.getPosition() -
                                            (int) Math.signum(robot.turret.getPosition()) - 8 / 255f);
                                })
                        )
                ),
                new Wait(1400)
        );
    }

    public static ICommand shootFromGate(Robot robot, int iteration, boolean sort) {
        return new Parallel(
                robot.drivetrain.followNext(d -> d.velocityCondition() && d.tValueCondition(0.8), 3000),
                new Sequential(
                        new Instant(robot.intakeMotor::outtakeMidSlow),
                        new Sequential(
                                robot.popper.pop(),
                                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.80)),
                                !sort ? robot.autoFastShoot() : robot.sortAndShootAuto()
                        )
                )
        );
    }

    public static ICommand intakeForSort(Robot robot, int iteration) {
        float turretPos = switch (iteration) {
            case 0 -> ServoTurret.AUTO_SET_1;
            case 1 -> ServoTurret.AUTO_SET_2;
            default -> ServoTurret.AUTO_SET_3;
        };
        int driveTimeout = iteration == 2 ? 4500 : 3000;

        return new Sequential(
                new Parallel(
                        new Sequential(
                                new Wait(300),
                                new Instant(() -> {
                                    robot.turret.setPosition(turretPos);
                                    robot.intakeMotor.intake();
                                    robot.flywheel.stop();
                                })
                        ),
                        resetTable(robot),
                        iteration == 0 ? new Sequential(
                                new Wait(600),
                                robot.drivetrain.followNext(d -> d.tValueCondition(0.8) && d.velocityCondition(), driveTimeout)
                        ) : robot.drivetrain.followNext(d -> d.tValueCondition(0.8) && d.velocityCondition(), driveTimeout)
                )
        );
    }

    public static ICommand shootForSort(ElapsedTime overallTimer, Robot robot, int iteration) {
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
                        new Conditional(
                                () -> iteration == 0,
                                Commands.NOOP,
                                new Sequential(
                                        new Wait(200),
                                        new Instant(robot.intakeMotor::outtakeSlow)
                                )
                        ),
                        new Instant(() -> {
                            robot.hood.setPosition(Hood.MEDIUM_PRESET - 10/255f);
                            robot.flywheel.setVelocity(Flywheel.CLOSE_AUTO_VELOCITY + flywheelOffset+10);
                            robot.tableCompartments.populate(intookColors);
                        }),
                        new Sequential(
                                new Race(
                                        new WaitUntil(() -> iteration == 0),
                                        new Wait(300),
                                        new WaitUntil(() -> iteration == 2 && overallTimer.seconds() > 27)
                                ),
                                new Instant(robot.intakeMotor::intake),
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

    public static ICommand intakePreload(Robot robot, boolean isFirst) {
        return new Sequential(
                new Instant(() -> {
                    if (!isFirst) {
                        robot.turret.setPosition(ServoTurret.UNSORTED_FINAL - 1/255f);
                        robot.hood.unsortedAuto();
                        robot.flywheel.setVelocity(Flywheel.NEAR_VELOCITY - 10);
                    }

                    robot.intakeMotor.intake();
                }),
                new Parallel(
                        resetTable(robot),
                        new Sequential(
                                new Wait(600),
                                robot.drivetrain.followNext(d -> d.tValueCondition(0.8) &&
                                        d.velocityCondition(), 3000),
                                isFirst ? new Instant(robot.flywheel::unsortedAuto) : Commands.NOOP,
                                new Sequential(
                                        new WaitUntil(() -> robot.drivetrain.tValueCondition(0.5)),
                                        new Instant(() -> robot.turret.unsortedAutoSet(0))
                                )
                        )
                )
        );
    }

    public static ICommand shootPreload(Robot robot, boolean isFirst) {
        return new Sequential(
                new Instant(() -> {
                    if (isFirst) {
                        robot.flywheel.unsortedAuto();
                        robot.hood.unsortedAuto();
                    } else {
                        robot.flywheel.setVelocity(Flywheel.NEAR_VELOCITY - 30);
                        robot.hood.setPosition(Hood.NEAR_PRESET - 10/255f);
                    }
                }),
                isFirst ? new Parallel(
                        robot.drivetrain.followNext(d -> d.velocityCondition() && d.tValueCondition(0.8), 3000),
                        preloadShootArmActions(robot)
                ) : new Deadline(
                        preloadShootArmActions(robot),
                        robot.drivetrain.followNext(d -> d.velocityCondition() && d.tValueCondition(0.8), 3000)
                )
        );
    }

    private static ICommand preloadShootArmActions(Robot robot) {
        return new Sequential(
                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.25)),
                robot.popper.pop(),
                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.9)),
                robot.autoFastShoot()
        );
    }

    @Override
    public void end() {
        if (!transferredData) Globals.turretStartPos = robot.turret.getPosition();
        //Drivetrain.startPose = robot.drivetrain.follower.getPose();
    }
}
