package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Popper;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.mechanisms.vision.DecodeLimelight;
import org.firstinspires.ftc.teamcode.opmodes.paths.CloseAutoPaths;
import org.firstinspires.ftc.teamcode.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.ArtifactColor;
import org.firstinspires.ftc.teamcode.utils.Globals;

public class CloseAuto extends OpModeCommand {
    private Robot robot;
    private ElapsedTime overallTimer;

    public CloseAuto(AllianceColor allianceColor) {
        Globals.allianceColor = allianceColor;
    }

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, new CloseAutoPaths());
        DecodeLimelight limelight = new DecodeLimelight(hardwareMap);
        robot.tableCompartments.populate(new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE});
        overallTimer = new ElapsedTime();

        schedule(
            infinite(robot::update),
            sequential(
                    waitUntil(() -> !opModeInInit()),
                    instant(overallTimer::reset)
            ),
            sequential(
                    instant(() -> {
                        robot.flywheel.auto();
                        robot.hood.near();
                        limelight.setCurrentPipeline(DecodeLimelight.Pipeline.OBELISK);
                    }),
                    deadline(
                            robot.drivetrain.followNext(d -> d.velocityCondition(4), 3000),
                            infinite(limelight::update)
                    ),
                    race(
                          Command.build()
                                  .setExecute(limelight::update)
                                  .setDone(() -> Globals.randomizationState != null),
                          waitMs(4000.0)
                    ),
                    instant(() -> robot.turret.setTargetPosition(Turret.AUTO_PRELOADS)),
                    robot.sortAndShoot(),
                    intake(0),
                    shoot(0),
                    intake(1),
                    shoot(1),
                    intake(2),
                    shoot(2),
                    park()
            )
        );
    }

    @Override
    public void end() {
        Drivetrain.startPose = robot.drivetrain.follower.getPose();
        Turret.startPos = robot.turret.getTargetPosition();
    }

    private Command intake(int iteration) {
        int turretPos = switch (iteration) {
            case 0 -> Turret.AUTO_SET_1;
            case 1 -> Turret.AUTO_SET_2;
            default -> Turret.AUTO_SET_3;
        };
        int driveTimeout = iteration == 2 ? 4500 : 3000;

        return sequential(
                instant(() -> {
                    robot.turret.setTargetPosition(turretPos);
                    robot.flywheel.stop();
                }),
                parallel(
                        robot.resetTableAfterShooting(),
                        robot.drivetrain.followNext(d -> d.tValueCondition(0.8) && d.velocityCondition(), driveTimeout)
                ),
                instant(robot.intakeMotor::outtakeSlow),
                waitMs(350.0),
                instant(robot.intakeMotor::intake),
                waitMs(100.0)
        );
    }

    private Command shoot(int iteration) {
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

        boolean isSorting = iteration != 2 && Globals.randomizationState != null;

        return sequential(
                parallel(
                        robot.drivetrain.followNext(d -> d.velocityCondition() && d.tValueCondition(0.8), driveTimeout),
                        conditional(
                                () -> iteration == 0,
                                Command.NOOP,
                                sequential(
                                        waitMs(200.0),
                                        instant(() -> robot.intakeMotor.outtakeSlow())
                                )
                        ),
                        instant(() -> {
                                robot.flywheel.runToVel(Flywheel.AUTO_VELOCITY + flywheelOffset);
                                robot.tableCompartments.populate(intookColors);
                        }),
                        sequential(
                                race(
                                        waitUntil(() -> iteration == 0),
                                        waitMs(300.0),
                                        waitUntil(() -> iteration == 2 && overallTimer.seconds() > 27)
                                ),
                                instant(() -> robot.intakeMotor.intake()),
                                race(
                                        waitUntil(() -> robot.drivetrain.tValueCondition(0.75)),
                                        waitUntil(() -> iteration == 2 && overallTimer.seconds() > 27.5
                                                    && overallTimer.seconds() < 28.7 && robot.drivetrain.distanceFromTarget() < 7)
                                ),
                                race(
                                        waitUntil(() -> !isSorting),
                                        robot.sort()
                                ),
                                waitMs(150.0),
                                robot.popper.pop(),
                                waitMs(250.0)
                        )
                ),
                robot.shootAll(() -> !isSorting ? 0.0 : 175.0)
        );
    }

    public Command park() {
        return parallel(
                robot.drivetrain.followNext(d -> d.velocityCondition(4)),
                instant(() -> {
                    robot.turret.resetTurret();
                    robot.flywheel.stop();
                    robot.intakeMotor.stop();
                }),
                conditional(
                        () -> robot.popper.atState(Popper.PopperState.NEUTRAL),
                        robot.table.reset(),
                        sequential(
                                robot.popper.neutral(),
                                robot.table.reset()
                        )
                )
        );
    }
}
