package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Deadline;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Popper;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret;
import org.firstinspires.ftc.teamcode.utils.commands.ArtifactColor;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.commands.Lazy;

import java.util.concurrent.atomic.AtomicBoolean;

public class AutoMethods {
    public static ICommand intakeFromGate(Robot robot, int iteration) {
        return new Sequential(
                new Instant(() -> {
                    robot.flywheel.stop();
                    robot.intakeMotor.stop();
                }),
                new Parallel(
                        resetTableBlock(robot),
                        new Sequential(
                                robot.drivetrain.followNext(d -> d.tValueCondition(0.8) && d.velocityCondition(), 3000),
                                new Instant(() -> {
                                    robot.turret.unsortedAutoSet(iteration + 1);
                                    robot.flywheel.unsortedAuto();
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
                new Wait(1200)
        );
    }

    public static ICommand resetTableBlock(Robot robot) {
        return new Sequential(
                //new Instant(robot.intakeMotor::stop),
                new Instant(() -> robot.table.setStateCommandless(Table.RelativeState.BALL1)),
                new Wait(650),
                robot.popper.blockFromPop()
        );
    }

    public static ICommand intakePreload(Robot robot, boolean isFirst) {
        return new Sequential(
                new Instant(() -> {
                    if (!isFirst) {
                        robot.turret.setPosition(ServoTurret.UNSORTED_FINAL);
                        robot.flywheel.setVelocity(Flywheel.NEAR_VELOCITY - 10);
                    }

                    robot.intakeMotor.intake();
                }),
                new Parallel(
                        resetTableBlock(robot),
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
                        new Sequential(
                                robot.popper.neutral(),
                                preloadShootArmActions(robot, true)
                        )
                ) : new Deadline(
                        new Sequential(
                                robot.popper.neutral(),
                                preloadShootArmActions(robot, true)
                        ),
                        robot.drivetrain.followNext(d -> d.velocityCondition() && d.tValueCondition(0.8), 3000)
                )
        );
    }

    public static ICommand park(Robot robot) {
        return new Parallel(
                robot.drivetrain.followNext(d -> d.velocityCondition(4)),
                new Instant(() -> {
                    robot.flywheel.stop();
                    robot.intakeMotor.stop();
                    Globals.turretStartPos = robot.turret.getPosition();
                }),
                new Conditional(
                        () -> robot.popper.atState(Popper.PopperState.NEUTRAL),
                        robot.table.reset(),
                        new Sequential(
                                robot.popper.neutral(),
                                robot.table.reset()
                        )
                )
        );
    }

    private static ICommand preloadShootArmActions(Robot robot, boolean isFirst) {
        return isFirst ? new Sequential(
                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.5)),
                robot.popper.pop(),
                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.80)),
                robot.autoFastShoot()
        ) : new Sequential(
                new Wait(200),
                new Instant(robot.intakeMotor::outtakeSlow),
                new Wait(280),
                new Instant(robot.intakeMotor::intake),
                new Wait(150),
                robot.popper.pop(),
                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.80)),
                new Wait(300),
                robot.autoFastShoot()
        );
    }

    public static ICommand resetTable(Robot robot) {
        return new Sequential(
                //new Instant(robot.intakeMotor::stop),
                robot.popper.neutral(),
                new Instant(() -> robot.table.setStateCommandless(Table.RelativeState.BALL1)),
                new Wait(650)
        );
    }

    public static ICommand shootFromGate(Robot robot, int iteration, boolean sort) {
        return new Parallel(
                robot.drivetrain.followNext(d -> d.velocityCondition() && d.tValueCondition(0.8), 3000),
                new Sequential(
                        new Instant(robot.intakeMotor::outtakeMidSlow),
                        new Sequential(
                                new Parallel(
                                        robot.popper.neutral(),
                                        new Sequential(
                                                new Wait(280),
                                                new Instant(robot.intakeMotor::intake),
                                                new Wait(300)
                                        )
                                ),
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
                        iteration == 2 ? robot.resetTable() : resetTableBlock(robot),
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
                        iteration != 2 ? robot.popper.neutral() : Commands.NOOP,
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
}
