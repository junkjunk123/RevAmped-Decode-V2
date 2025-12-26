package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.ivy.Command;
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
import org.firstinspires.ftc.teamcode.mechanisms.DecodeLimelight;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Popper;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.opmodes.paths.CloseAutoPaths;
import org.firstinspires.ftc.teamcode.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.ArtifactColor;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.commands.Functional;

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
            new Infinite(robot::update),
            new Sequential(
                    new WaitUntil(() -> !opModeInInit()),
                    new Instant(overallTimer::reset)
            ),
            new Sequential(
                    new Instant(() -> {
                        robot.flywheel.auto();
                        robot.hood.near();
                        limelight.setCurrentPipeline(DecodeLimelight.Pipeline.OBELISK);
                    }),
                    new Deadline(
                            robot.drivetrain.followNext(d -> d.velocityCondition(4), 3000),
                            new Infinite(limelight::update)
                    ),
                    new Race(
                          new Functional(() -> {}, limelight::update, () -> Globals.randomizationState != null),
                          new Wait(4000)
                    ),
                    new Instant(() -> robot.turret.setTargetPosition(Turret.AUTO_PRELOADS)),
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
                }),
                new Parallel(
                        robot.resetAfterShooting(),
                        robot.drivetrain.followNext(d -> d.tValueCondition(0.8) && d.velocityCondition(), driveTimeout)
                ),
                new Instant(robot.intakeMotor::outtakeSlow),
                new Wait(350),
                new Instant(robot.intakeMotor::intake),
                new Wait(100)
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

        boolean isSorting = iteration != 2 && Globals.randomizationState != null;

        return new Sequential(
                new Parallel(
                        robot.drivetrain.followNext(d -> d.velocityCondition() && d.tValueCondition(0.8), driveTimeout),
                        new Conditional(
                                () -> iteration == 0,
                                new Command(),
                                new Sequential(
                                        new Wait(200),
                                        new Instant(() -> robot.intakeMotor.outtakeSlow())
                                )
                        ),
                        new Instant(() -> {
                                robot.flywheel.setTargetVelocity(Flywheel.AUTO_VELOCITY + flywheelOffset);
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
                                new Race(
                                        new WaitUntil(() -> !isSorting),
                                        robot.sort()
                                ),
                                new Wait(150),
                                new Instant(() -> robot.popper.pop()),
                                new Wait(250)
                        )
                ),
                robot.shootAll(() -> !isSorting ? 0.0 : 175.0)
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
                        () -> robot.popper.atPos(Popper.NEUTRAL),
                        robot.table.reset(),
                        new Sequential(
                                new Instant(robot.popper::neutral),
                                new Wait(250),
                                new Instant(robot.table::reset)
                        )
                )
        );
    }
}
