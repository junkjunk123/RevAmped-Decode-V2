package org.firstinspires.ftc.teamcode.opmodes;

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
import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.commands.Functional;
import org.firstinspires.ftc.teamcode.utils.commands.Lazy;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public class CloseAuto extends OpModeCommand {
    private Robot robot;
    private ElapsedTime overallTimer;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, new CloseAutoPaths());
        DecodeLimelight limelight = new DecodeLimelight(hardwareMap);
        robot.tableCompartments.populate(new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE});
        overallTimer = new ElapsedTime();

        schedule(
            new Infinite(robot::update),
            new Sequential(
                    new WaitUntil(() -> !opModeInInit()),
                    new Instant(overallTimer::reset),
                    new Instant(() -> {
                        robot.flywheel.closeAuto();
                        robot.hood.near();
                        robot.intakeMotor.intakeSlow();
                        limelight.setCurrentPipeline(DecodeLimelight.Pipeline.OBELISK);
                    }),
                    new Race(
                            robot.drivetrain.followNext(d -> d.velocityCondition(4), 3000),
                            new Infinite(limelight::update)
                    ),
                    new Race(
                          new Functional(() -> {}, limelight::update, () -> Globals.randomizationState != null),
                          new Wait(4000)
                    ),
                    new Instant(() -> {
                        limelight.close();
                        robot.turret.setTargetPosition(Turret.AUTO_PRELOADS);
                        robot.intakeMotor.intakeSlow();
                    }),
                    new Lazy(() -> {
                        if (Globals.randomizationState != null)
                            return robot.sortAndShootAuto();
                        return new Sequential(
                                robot.popper.pop(),
                                robot.shootAll()
                        );
                    }),
                    new Wait(600),
                    intake(0),
                    new Parallel(
                            new Sequential(
                                    new Instant(robot.intakeMotor::outtakeSlow),
                                    new Wait(200),
                                    new Instant(robot.intakeMotor::intake)
                            ),
                            shoot(0)
                    ),
                    intake(1),
                    new Parallel(
                            new Sequential(
                                    new Instant(robot.intakeMotor::outtakeSlow),
                                    new Wait(200),
                                    new Instant(robot.intakeMotor::intake)
                            ),
                            shoot(1)
                    ),
                    intake(2),
                    new Parallel(
                            new Sequential(
                                    new Instant(robot.intakeMotor::outtakeSlow),
                                    new Wait(200),
                                    new Instant(robot.intakeMotor::intake)
                            ),
                            shoot(2)
                    ),
                    park()
            )
        );
    }

    @Override
    public void execute() {
        List<Double> powers = new ArrayList<>();
        robot.drivetrain.apply(m -> powers.add(m.getPower()));
        telemetry.addData("powers", Arrays.toString(powers.toArray()));
        telemetry.addData("limelightDetected", Globals.randomizationState != null);
        if (Globals.randomizationState != null) telemetry.addData("motif", Globals.randomizationState);
        telemetry.update();
    }

    @Override
    public void end() {
        Drivetrain.startPose = robot.drivetrain.follower.getPose();
        Turret.startPos = robot.turret.getTargetPosition();
    }

    private ICommand intake(int iteration) {
        int turretPos = switch (iteration) {
            case 0 -> Turret.AUTO_SET_1;
            case 1 -> Turret.AUTO_SET_2;
            default -> Turret.AUTO_SET_3;
        };
        int driveTimeout = iteration == 2 ? 4500 : 3000;

        return new Sequential(
                new Instant(() -> {
                    robot.turret.setTargetPosition(turretPos);
                    robot.flywheel.stop();
                    robot.intakeMotor.intake();
                }),
                new Parallel(
                        robot.resetTable(),
                        robot.drivetrain.followNext(d -> d.tValueCondition(0.8) && d.velocityCondition(), driveTimeout)
                )
        );
    }

    private ICommand shoot(int iteration) {
        double flywheelOffset = iteration == 2 ? 10 : 0;

        double driveTimeout = switch (iteration) {
            case 0 -> 3000;
            case 1 -> 2700;
            default -> 3500;
        };

        ArtifactColor[] intookColors = switch (iteration) {
            case 0 -> new ArtifactColor[] {ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE};
            case 1 -> new ArtifactColor[] {ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.GREEN};
            default -> new ArtifactColor[] {ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE};
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
                robot.autoSlowShoot(() -> !isSorting.get() ? 0.0 : Table.SLOW_SHOOT_DELAY)
        );
    }

    public ICommand park() {
        return new Parallel(
                robot.drivetrain.followNext(d -> d.velocityCondition(4)),
                new Instant(() -> {
                    robot.turret.resetTurret();
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
                )
        );
    }
}
