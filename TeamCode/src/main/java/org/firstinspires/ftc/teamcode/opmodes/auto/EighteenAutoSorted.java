package org.firstinspires.ftc.teamcode.opmodes.auto;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.opmodes.paths.EighteenPaths;
import org.firstinspires.ftc.teamcode.utils.ArtifactColor;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RandomizationState;
import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Lazy;

import java.util.function.BooleanSupplier;

@Autonomous
public class EighteenAutoSorted extends OpModeCommand {
    private Robot robot;
    private final ElapsedTime overallTimer = new ElapsedTime();
    private static boolean testSlowShoot = false;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, new EighteenPaths());
        robot.tableCompartments.populate(ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE);
        robot.turret.setPosition(ServoTurret.EIGHTEEN_DETECTION);

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
                            robot.flywheel.far();
                            robot.hood.far();
                            robot.feederWheel.start();
                            robot.popper.pop();
                        }),
                        new Parallel(
                                new Sequential(
                                        !testSlowShoot ? new Race(
                                                robot.limelight.detectMotif(),
                                                new Wait(3000)
                                        ) : new Instant(() -> Globals.randomizationState = RandomizationState.GPP),
                                        new Instant(() -> robot.turret.setPosition(ServoTurret.EIGHTEEN_PRELOADS)),
                                        new Wait(400)
                                ),
                                new Wait(800)
                        ),
                        robot.autoFastShoot(),
                        new Parallel(
                                intake(false),
                                robot.drivetrain.follow()
                        ),
                        new Parallel(
                                robot.drivetrain.follow(),
                                new Sequential(
                                        new Wait(300),
                                        transfer()
                                ),
                                new Instant(() -> robot.turret.setPosition(ServoTurret.EIGHTEEN_FIRST_SET))
                        ),
                        robot.autoFastShoot(),
                        new Parallel(
                                intake(true),
                                robot.drivetrain.follow()
                        ),
                        new Parallel(
                                robot.drivetrain.follow(),
                                new Instant(() -> {
                                    robot.hood.medium();
                                    robot.flywheel.medium();
                                    robot.feederWheel.start();
                                    robot.turret.setPosition(ServoTurret.EIGHTEEN_SECOND_SET);
                                }),
                                transfer()
                        ),
                        robot.autoFastShoot(),
                        new Parallel(
                                intakeFromGate(),
                                robot.drivetrain.follow()
                        ),
                        new Parallel(
                                robot.drivetrain.follow(),
                                shootFromGate(() -> Globals.randomizationState != null)
                        ),
                        new Parallel(
                                intake(true),
                                robot.drivetrain.follow(),
                                new Instant(() -> robot.turret.setPosition(ServoTurret.EIGHTEEN_THIRD_SET))
                        ),
                        new Parallel(
                                robot.drivetrain.follow(),
                                transferSorted(),
                                new Instant(() -> {
                                    robot.flywheel.medium();
                                    robot.feederWheel.start();
                                })
                        ),
                        robot.shootAll(() -> Globals.randomizationState != null ? 0.0 : Table.SLOW_SHOOT_DELAY),
                        new Parallel(
                                intake(true),
                                robot.drivetrain.follow(),
                                new Instant(() -> robot.turret.setPosition(ServoTurret.EIGHTEEN_FOURTH_SET))
                        ),
                        new Parallel(
                                robot.drivetrain.follow(),
                                transferSorted(),
                                new Instant(() -> {
                                    robot.flywheel.medium();
                                    robot.feederWheel.start();
                                })
                        ),
                        robot.shootAll(() -> Globals.randomizationState != null ? 0.0 : Table.SLOW_SHOOT_DELAY),
                        robot.drivetrain.follow()
                )
        );
    }

    public ICommand intake(boolean stopShooter) {
        return new Sequential(
                new Instant(() -> {
                    if (stopShooter) {
                        robot.flywheel.stop();
                        robot.feederWheel.stop();
                    }
                }),
                new Instant(() -> robot.table.setStateCommandless(Table.RelativeState.BALL1)),
                new Wait(650),
                robot.intake()
        );
    }

    public ICommand transfer() {
        return new Parallel(
                new Instant(() -> robot.intakeTilt.transfer()),
                new Sequential(
                        new Wait(200),
                        new Instant(robot.intakeMotor::outtake)
                ),
                robot.popper.pop(),
                robot.splitter.neutral(),
                robot.intakeGate.close()
        );
    }

    public ICommand transferSorted() {
        return new Sequential(
                new Instant(() -> robot.intakeTilt.transfer()),
                new Sequential(
                        new Wait(200),
                        new Instant(robot.intakeMotor::outtake)
                ),
                new Parallel(
                        robot.splitter.neutral(),
                        robot.intakeGate.close()
                ),
                new Lazy(() -> {
                    if (Globals.randomizationState != null) {
                        return new Sequential(
                                new Race(
                                        robot.tableCompartments.populateAuto(),
                                        new Wait(300)
                                ),
                                robot.sortAuto()
                        );
                    }
                    return Commands.NOOP;
                }),
                new Instant(robot.intakeMotor::stop),
                robot.popper.pop()
        );
    }

    public ICommand intakeFromGate() {
        return new Sequential(
                new Instant(() -> {
                    robot.flywheel.stop();
                    robot.feederWheel.stop();
                }),
                new Parallel(
                        new Instant(() -> {
                            robot.turret.setPosition(ServoTurret.EIGHTEEN_GATE_SHOOT +
                                    (int) Math.signum(ServoTurret.EIGHTEEN_GATE_SHOOT) * 8 / 255f);
                            robot.flywheel.medium();
                            robot.table.setStateCommandless(Table.RelativeState.BALL1);
                        }),
                        new Sequential(
                                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.75)),
                                new Instant(() -> {
                                    robot.turret.setPosition(robot.turret.getPosition() -
                                        (int) Math.signum(robot.turret.getPosition()) - 8 / 255f);
                                    robot.intake();
                                })
                        )
                ),
                new Race(
                        new Wait(1200),
                        new WaitUntil(() -> robot.intakeDistance.hasArtifact())
                )
        );
    }

    public ICommand shootFromGate(BooleanSupplier isSort) {
        return new Sequential(
                new Instant(() -> {
                    robot.intakeTilt.transfer();
                    robot.intakeMotor.outtake();
                    robot.flywheel.medium();
                    robot.feederWheel.start();
                }),
                new Parallel(
                        robot.splitter.neutral(),
                        robot.intakeGate.close()
                ),
                new Parallel(
                        new Sequential(
                                new Instant(robot.intakeMotor::intake),
                                new Wait(200),
                                new Instant(robot.intakeMotor::stop)
                        ),
                        isSort.getAsBoolean() ? new Race(
                                robot.tableCompartments.populateAuto(),
                                new Wait(300)
                        ) : Commands.NOOP
                ),
                isSort.getAsBoolean() ? robot.sortAuto() : Commands.NOOP,
                robot.popper.pop(),
                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.8)),
                robot.autoShoot(() -> isSort.getAsBoolean() ? 0.0 : Table.SLOW_SHOOT_DELAY)
        );
    }
}
