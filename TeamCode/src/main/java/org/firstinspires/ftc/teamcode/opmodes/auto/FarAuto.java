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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.GyroThread;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.opmodes.paths.FarAutoPaths;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.FollowParameters;
import org.firstinspires.ftc.teamcode.utils.vision.BlobTransformer;
import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.commands.Lazy;
import org.firstinspires.ftc.teamcode.utils.commands.channel.Channel;
import org.firstinspires.ftc.teamcode.utils.commands.channel.Channels;
import org.firstinspires.ftc.teamcode.utils.commands.channel.Speaker;
import org.firstinspires.ftc.teamcode.utils.math.Z3Element;
import org.firstinspires.ftc.teamcode.utils.math.projectile.TrackState;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReferenceArray;

public class FarAuto extends OpModeCommand {
    private Robot robot;
    private GyroThread gyroThread;
    private final ElapsedTime overallTimer = new ElapsedTime();
    private Z3Element cyclePath = new Z3Element(-1);
    public static boolean useVision = false;
    private final AtomicInteger cameraFailures = new AtomicInteger(0);

    public static int MAX_CAM_FAILURES = 3;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, new FarAutoPaths());
        gyroThread = new GyroThread(robot);
        gyroThread.setState(TrackState.FAR_TWO);
        robot.hood.far();
        Constants.K_LINEAR_BRAKE = 0.090; Constants.K_QUADRATIC_BRAKE = 0.00125;

        schedule(
                new Infinite(() -> {
                    robot.update();
                    gyroThread.update(true);
                    Pose pose = robot.drivetrain.follower.getPose();
                    if (pose.distanceFrom(new Pose()) > 0.01) Drivetrain.startPose = robot.drivetrain.follower.getPose();
                    if (robot.intakeMotor.atState(IntakeMotor.IntakeState.INTAKE)) robot.tableCompartments.intakeThread.update();
                    telemetry.addData("balls", robot.tableCompartments.intakeThread.getNumBalls());
                    telemetry.update();
                }),
                new WaitUntil(() -> !opModeInInit()),
                new Instant(overallTimer::reset),
                new Sequential(
                        new Instant(() -> {
                            robot.flywheel.setVelocity(Flywheel.FAR_VELOCITY - 15);
                            robot.feederWheel.start();
                            robot.popper.popCommandless();
                        }),
                        new Wait(800),
                        shoot(),
                        new Parallel(
                                intake(),
                                robot.turret.resetTurret(),
                                new Sequential(
                                        new Wait(400),
                                        new Race(
                                                new WaitUntil(() -> robot.tableCompartments.intakeThread.hasThree),
                                                new Sequential(
                                                        robot.drivetrain.followNext(d -> d.velocityCondition(4) || d.follower.getCurrentTValue() >= 0.95, 3000),
                                                        new Wait(250)
                                                )
                                        )
                                )
                        ),
                        new Parallel(
                                transfer(),
                                new Instant(() -> gyroThread.setState(TrackState.FAR_AUTO, true)),
                                new Race(
                                        robot.drivetrain.followNext(d -> d.velocityCondition(4) || d.follower.getCurrentTValue() >= 0.95, 3000),
                                        new Sequential(
                                                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.9)),
                                                shoot()
                                        )
                                )
                        ),
                        new Parallel(
                                intake(),
                                robot.turret.resetTurret(),
                                new Race(
                                        new WaitUntil(() -> robot.tableCompartments.intakeThread.hasThree),
                                        new Sequential(
                                                robot.drivetrain.followNext(d -> d.velocityCondition(4) || d.follower.getCurrentTValue() >= 0.95, 3000),
                                                new Wait(250)
                                        )
                                )
                        ),
                        new Parallel(
                                robot.drivetrain.followNext(d -> d.velocityCondition(4) || d.follower.getCurrentTValue() >= 0.95, 3000),
                                new Instant(() -> gyroThread.setState(TrackState.FAR_AUTO, true)),
                                transfer(),
                                new Sequential(
                                        new WaitUntil(() -> robot.drivetrain.tValueCondition(0.9)),
                                        shoot()
                                )
                        ),
                        cycle(),
                        cycle(),
                        cycle(),
                        cycle(),
                        cycle(),
                        cycle(),
                        cycle(),
                        cycle(),
                        cycle(),
                        cycle(),
                        cycle(),
                        cycle(),
                        cycle(),
                        cycle(),
                        cycle(),
                        cycle(),
                        cycle()
                )
        );
    }

    public ICommand cycleWithoutVision() {
        return new Sequential(
                new Parallel(
                        intake(),
                        robot.turret.resetTurret(),
                        new Race(
                                new WaitUntil(() -> robot.tableCompartments.intakeThread.hasThree),
                                new Sequential(
                                        robot.drivetrain.followNext(d -> d.velocityCondition(4) || d.follower.getCurrentTValue() >= 0.95, 3000),
                                        new Wait(250)
                                )
                        )
                ),
                new Parallel(
                        robot.drivetrain.followNext(d -> d.velocityCondition(6) || d.follower.getCurrentTValue() >= 0.95, 3000),
                        new Instant(() -> gyroThread.setState(TrackState.FAR_AUTO, true)),
                        transfer(),
                        new Sequential(
                                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.9)),
                                shoot()
                        )
                )
        );
    }

    public ICommand cycle() {
        return new Conditional(
                () -> useVision && cameraFailures.get() <= MAX_CAM_FAILURES,
                visionCycle(),
                cycleWithoutVision()
        );
    }

    public ICommand visionCycle() {
        Channel<Integer> selectedCycle = Channels.oneshot();
        AtomicReferenceArray<FollowParameters> selectedPaths = new AtomicReferenceArray<>(2);

        return new Sequential(
                new Parallel(
                        getPathFromVision().subscribe(selectedCycle),
                        intake(),
                        new Race(
                                new WaitUntil(() -> robot.tableCompartments.intakeThread.hasThree),
                                new Sequential(
                                        new Race(
                                                new Sequential(
                                                        selectedCycle.listen(),
                                                        new Instant(() -> {
                                                            Integer selected = Channels.receive(selectedCycle);
                                                            if (selected == null) throw new IllegalArgumentException("Camera short-circuited trust not the code");
                                                            FollowParameters[] cycle = FarAutoPaths.getCycle(selected, robot.drivetrain);
                                                            selectedPaths.set(0, cycle[0]);
                                                            selectedPaths.set(1, cycle[1]);
                                                        })
                                                ),
                                                new Sequential(
                                                        new Wait(400),
                                                        new Instant(() -> {
                                                            FollowParameters[] cycle = FarAutoPaths.getDefaultCycle(robot.drivetrain);
                                                            selectedPaths.set(0, cycle[0]);
                                                            selectedPaths.set(1, cycle[1]);
                                                            cameraFailures.getAndIncrement();
                                                        })
                                                )
                                        ),
                                        selectedPaths.get(0).followCommand(robot.drivetrain),
                                        new Wait(250)
                                )
                        )
                ),
                new Parallel(
                        selectedPaths.get(1).followCommand(robot.drivetrain),
                        new Instant(() -> gyroThread.setState(TrackState.FAR_AUTO, true)),
                        transfer(),
                        new Sequential(
                                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.9)),
                                shoot()
                        )
                )
        );
    }

    public Speaker<Integer> getPathFromVision() {
        return new Speaker<>(c -> new Sequential(
                new Instant(robot.intakeCamera::start),
                new WaitUntil(robot.intakeCamera::hasBlobs),
                Channels.send(c, () -> BlobTransformer.computeIntakeRegion(robot.intakeCamera.getAllBlobs(), robot.drivetrain.follower.getHeading())),
                new Instant(robot.intakeCamera::stop)
        ));
    }

    private int selectPath() {
        cyclePath = cyclePath.plus(1);
        return cyclePath.getVal();
    }

    private ICommand followCycle(AtomicInteger i) {
        return new Lazy(
                () -> new Sequential(
                        new Instant(() -> {
                            int chosenCycle = i.get();
                            robot.drivetrain.skip(chosenCycle);
                        }),
                        robot.drivetrain.followNext(d -> d.velocityCondition(4) || d.follower.getCurrentTValue() >= 0.95, 3000),
                        new Instant(() -> robot.drivetrain.skip(2 - i.get()))
                )
        );
    }

    public ICommand shoot() {
        return new Sequential(
                new Instant(() -> {
                    robot.intakeMotor.stop();
                    /*
                    robot.drivetrain.follower.useTranslational = false;
                    robot.drivetrain.follower.useDrive = false;
                    robot.drivetrain.follower.useHeading = false;
                     */
                }),
                new Parallel(
                        robot.autoFastShoot(),
                        new Sequential(
                                new Wait(150),
                                new Instant(() -> GyroThread.NEUTRAL_OFFSET = 0)
                        )
                ),
                new Instant(() -> {
                    robot.drivetrain.follower.useTranslational = true;
                    robot.drivetrain.follower.useDrive = true;
                    robot.drivetrain.follower.useHeading = true;
                })
        );
    }

    public ICommand intake() {
        return new Sequential(
                new Instant(() -> {
                    robot.flywheel.stop();
                    robot.feederWheel.intakeState();
                }),
                new Instant(() -> robot.table.setStateCommandless(Table.RelativeState.BALL1)),
                new Wait(300),
                new Parallel(
                        new Sequential(
                                new Wait(300),
                                robot.popper.neutral()
                        ),
                        robot.intake()
                )
        );
    }

    public ICommand transfer() {
        return new Sequential(
                new Conditional(
                        () -> robot.tableCompartments.intakeThread.hasThree,
                        Commands.NOOP,
                        new Wait(350)
                ),
                new Instant(() -> {
                    robot.intakeTilt.transfer();
                    robot.intakeMotor.outtake();
                    GyroThread.NEUTRAL_OFFSET = 2/255d;
                }),
                new Parallel(
                        robot.popper.pop(),
                        robot.splitter.neutral(),
                        robot.intakeGate.close(),
                        new Instant(() -> robot.feederWheel.start())
                )
        );
    }
}
