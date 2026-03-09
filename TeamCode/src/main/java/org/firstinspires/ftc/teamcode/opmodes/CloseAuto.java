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

import java.util.concurrent.atomic.AtomicBoolean;

public class CloseAuto extends OpModeCommand {
    private Robot robot;
    private ElapsedTime overallTimer;
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
                    new Race(
                            robot.drivetrain.followNext(d -> d.velocityCondition(4), 3000),
                            new Sequential(
                                    !testSlowShoot ? new Functional(() -> {}, limelight::update, () -> Globals.randomizationState != null) :
                                            new Instant(() -> Globals.randomizationState = RandomizationState.PPG),
                                    new Instant(() -> robot.turret.setTargetPosition(Turret.AUTO_PRELOADS)),
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
                        robot.turret.setTargetPosition(Turret.AUTO_PRELOADS);
                        robot.intakeMotor.intakeSlow();
                    }),
                    new Lazy(() -> {
                        if (Globals.randomizationState != null)
                            return robot.sortAndShootAuto();
                        return new Sequential(
                                robot.popper.pop(),
                                robot.autoFastShoot()
                        );
                    }),
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
        telemetry.addData("limelightDetected", Globals.randomizationState != null);
        if (Globals.randomizationState != null) telemetry.addData("motif", Globals.randomizationState);
        telemetry.update();
    }

    @Override
    public void end() {
        //Drivetrain.startPose = robot.drivetrain.follower.getPose();
    }

    private ICommand intake(int iteration) {
        return AutoMethods.intakeForSort(robot, iteration);
    }

    private ICommand shoot(int iteration) {
        return AutoMethods.shootForSort(overallTimer, robot, iteration);
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
}
