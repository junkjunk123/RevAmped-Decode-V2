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
import org.firstinspires.ftc.teamcode.pedro.FollowParameters;
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
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.atomic.AtomicReferenceArray;

public class FarAuto extends OpModeCommand {
    private Robot robot;
    private GyroThread gyroThread;
    private final ElapsedTime overallTimer = new ElapsedTime();
    private Z3Element cyclePath = new Z3Element(-1);

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, new FarAutoPaths());
        gyroThread = new GyroThread(robot);
        gyroThread.setState(TrackState.FAR_ONE);
        robot.hood.far();

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
                                new Sequential(
                                        new Wait(400),
                                        new Race(
                                                new WaitUntil(() -> robot.tableCompartments.intakeThread.getNumBalls() == 3),
                                                robot.drivetrain.follow()
                                        )
                                )
                        ),
                        new Wait(250),
                        new Parallel(
                                robot.drivetrain.follow(),
                                new Sequential(
                                        new Wait(200),
                                        transfer()
                                ),
                                new Instant(() -> gyroThread.setState(TrackState.FAR_AUTO, true))
                        ),
                        shoot(),
                        new Parallel(
                                intake(),
                                new Race(
                                        new WaitUntil(() -> robot.tableCompartments.intakeThread.getNumBalls() == 3),
                                        robot.drivetrain.follow()
                                )
                        ),
                        new Wait(250),
                        new Parallel(
                                robot.drivetrain.follow(),
                                new Instant(() -> gyroThread.setState(TrackState.FAR_AUTO, true)),
                                new Sequential(
                                        new Wait(200),
                                        transfer()
                                )
                        ),
                        shoot(),
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

    public ICommand cycle() {
        return new Sequential(
                new Parallel(
                        intake(),
                        new Race(
                                new WaitUntil(() -> robot.tableCompartments.intakeThread.getNumBalls() == 3),
                                robot.drivetrain.follow()
                        )
                ),
                new Wait(250),
                new Parallel(
                        robot.drivetrain.follow(),
                        new Instant(() -> gyroThread.setState(TrackState.FAR_AUTO, true)),
                        new Sequential(
                                new Wait(200),
                                transfer()
                        )
                ),
                shoot()
        );
    }

    public ICommand visionCycle() {
        Channel<Integer> selectedCycle = Channels.oneshot();
        AtomicReferenceArray<FollowParameters> selectedPaths = new AtomicReferenceArray<>(2);
        AtomicBoolean foundPath = new AtomicBoolean();

        return new Sequential(
                new Parallel(
                        getPathFromVision().subscribe(selectedCycle),
                        intake(),
                        new Race(
                                new WaitUntil(() -> robot.tableCompartments.intakeThread.getNumBalls() == 3),
                                new Sequential(
                                        robot.drivetrain.follow(),
                                        new WaitUntil(foundPath::get)
                                ),
                                new Sequential(
                                        selectedCycle.listen(),
                                        new Instant(() -> {
                                            Integer selected = Channels.receive(selectedCycle);
                                            if (selected == null) throw new IllegalArgumentException("Camera short-circuited trust not the code");
                                            FollowParameters[] cycle = FarAutoPaths.getCycle(selected, robot.drivetrain);
                                            selectedPaths.set(0, cycle[0]);
                                            selectedPaths.set(1, cycle[1]);
                                            foundPath.set(true);
                                        }),
                                        selectedPaths.get(0).followCommand(robot.drivetrain)
                                )
                        )
                ),
                new Wait(250),
                new Parallel(
                        new Conditional(
                                foundPath::get,
                                selectedPaths.get(1).followCommand(robot.drivetrain),
                                robot.drivetrain.follow()
                        ),
                        robot.drivetrain.follow(),
                        new Instant(() -> gyroThread.setState(TrackState.FAR_AUTO, true)),
                        new Sequential(
                                new Wait(200),
                                transfer()
                        )
                ),
                shoot()
        );
    }

    public Speaker<Integer> getPathFromVision() {
        return new Speaker<>(c -> Commands.NOOP);
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
                        robot.drivetrain.follow(),
                        new Instant(() -> robot.drivetrain.skip(2 - i.get()))
                )
        );
    }

    public ICommand shoot() {
        return new Sequential(
                new Instant(() -> {
                    robot.intakeMotor.stop();
                    robot.drivetrain.follower.useTranslational = false;
                    robot.drivetrain.follower.useDrive = false;
                    robot.drivetrain.follower.useHeading = false;
                }),
                robot.autoFastShoot(),
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
                new Wait(300),
                new Instant(() -> {
                    robot.intakeTilt.transfer();
                    robot.intakeMotor.outtake();
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
