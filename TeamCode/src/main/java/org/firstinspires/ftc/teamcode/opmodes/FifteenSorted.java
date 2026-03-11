package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.AutoMethods.resetTableBlock;

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
import org.firstinspires.ftc.teamcode.mechanisms.intake.Popper;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.mechanisms.vision.DecodeLimelight;
import org.firstinspires.ftc.teamcode.opmodes.paths.CloseAutoPaths;
import org.firstinspires.ftc.teamcode.opmodes.paths.FifteenSortedPaths;
import org.firstinspires.ftc.teamcode.utils.AllianceColor;
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
    private static boolean testSlowShoot = false;

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
                            robot.turret.setTargetPosition(Turret.FIFTEEN_OBELISK_DETECTION);
                            robot.flywheel.setVelocity(Flywheel.MEDIUM_VELOCITY);
                            robot.hood.medium();
                            robot.intakeMotor.intakeSlow();
                            limelight.setCurrentPipeline(DecodeLimelight.Pipeline.OBELISK);
                        }),
                        new Race(
                                robot.drivetrain.followNext(d -> d.velocityCondition(4), 3000),
                                new Sequential(
                                        !testSlowShoot ? new Functional(() -> {}, limelight::update, () -> Globals.randomizationState != null) :
                                                new Instant(() -> Globals.randomizationState = RandomizationState.GPP),
                                        new Instant(() -> robot.turret.setTargetPosition(Turret.FIFTEEN_BALL_PRELOADS)),
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
                            robot.turret.setTargetPosition(Turret.FIFTEEN_BALL_PRELOADS);
                            robot.intakeMotor.intakeSlow();
                        }),
                        robot.popper.pop(),
                        robot.autoFastShoot(),
                        AutoMethods.intakePreload(robot, true),
                        AutoMethods.shootPreload(robot, true),
                        AutoMethods.intakeFromGate(robot, 0),
                        AutoMethods.shootFromGate(robot, 0, true),
                        AutoMethods.intakeForSort(robot, 0),
                        AutoMethods.shootForSort(overallTimer, robot, 0),
                        AutoMethods.intakeForSort(robot, 2),
                        intakeLastPreloadSet(),
                        shootLastPreloadSet(),
                        new Parallel(
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
                                    robot.turret.setTargetPosition(Turret.UNSORTED_FINAL+7);
                                    robot.intakeMotor.intake();
                                    robot.flywheel.stop();
                                })
                        ),
                        resetTableBlock(robot),
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
                        robot.popper.neutral(),
                        new Sequential(
                                new Instant(robot.intakeMotor::outtakeMidSlow),
                                new Wait(200)
                        ),
                        new Instant(() -> {
                            robot.flywheel.setVelocity(Flywheel.NEAR_VELOCITY);
                            robot.hood.setPosition(Hood.NEAR_PRESET - 12/255f);
                            robot.tableCompartments.populate(colors);
                        }),
                        new Sequential(
                                new Wait(300),
                                new Instant(robot.intakeMotor::intake),
                                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.75)),
                                new Lazy(() -> {
                                    if (Globals.randomizationState != null) return robot.sortAuto();
                                    return Commands.NOOP;
                                }),
                                new Wait(150),
                                robot.popper.pop()
                        )
                ),
                robot.autoShoot(() -> Globals.randomizationState == null ? 0.0 : Table.SLOW_SHOOT_DELAY)
        );
    }

    private ICommand shootSecondLastForSort() {
        double driveTimeout = 2700;
        ArtifactColor[] intakeColors = new ArtifactColor[] {ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE};

        return new Sequential(
                new Parallel(
                        robot.drivetrain.followNext(d -> d.velocityCondition() && d.tValueCondition(0.8), driveTimeout),
                        robot.popper.neutral(),
                        new Sequential(
                                new Wait(200),
                                new Instant(robot.intakeMotor::outtakeSlow)
                        ),
                        new Instant(() -> {
                            robot.hood.near();
                            robot.flywheel.setVelocity(Flywheel.CLOSE_AUTO_VELOCITY + 50);
                            robot.tableCompartments.populate(intakeColors);
                        }),
                        new Sequential(
                                new Instant(robot.intakeMotor::intake),
                                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.75)),
                                new Lazy(() -> {
                                    if (Globals.randomizationState != null) return robot.sortAuto();
                                    return Commands.NOOP;
                                }),
                                new Wait(150),
                                robot.popper.pop()
                        )
                ),
                robot.autoShoot(() -> Globals.randomizationState == null ? 0.0 : Table.SLOW_SHOOT_DELAY)
        );
    }
}
