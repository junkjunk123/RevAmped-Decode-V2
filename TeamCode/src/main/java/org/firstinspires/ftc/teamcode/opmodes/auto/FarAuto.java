package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.GyroThread;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.opmodes.paths.FarAutoPaths;
import org.firstinspires.ftc.teamcode.utils.commands.Lazy;
import org.firstinspires.ftc.teamcode.utils.commands.Loop;
import org.firstinspires.ftc.teamcode.utils.math.Z3Element;

import java.util.concurrent.atomic.AtomicInteger;

public class FarAuto extends OpModeCommand {
    private Robot robot;
    private GyroThread gyroThread;
    private final ElapsedTime overallTimer = new ElapsedTime();
    private Z3Element cyclePath;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, new FarAutoPaths());
        gyroThread = new GyroThread(robot);

        schedule(
                new Instant(overallTimer::reset),
                new Instant(() -> {
                    robot.flywheel.far();
                    robot.hood.far();
                    robot.feederWheel.start();
                    robot.popper.popCommandless();
                }),
                new Wait(800),
                robot.autoFastShoot(),
                new Parallel(
                        intake(false),
                        robot.drivetrain.follow(),
                        new Instant(() -> robot.flywheel.setVelocity(Flywheel.FAR_VELOCITY - 5))
                ),
                new Wait(300),
                new Parallel(
                        robot.drivetrain.follow(),
                        new Sequential(
                                new Wait(600),
                                transfer()
                        ),
                        new Instant(() -> {
                            robot.turret.setPosition(ServoTurret.EIGHTEEN_FIRST_SET);
                            robot.feederWheel.start();
                        })
                ),
                robot.autoFastShoot(),
                new Parallel(
                        intake(true),
                        robot.drivetrain.follow()
                ),
                new Parallel(
                        robot.drivetrain.follow(),
                        new Instant(() -> {
                            robot.feederWheel.start();
                            robot.turret.setPosition(ServoTurret.EIGHTEEN_SECOND_SET);
                        }),
                        transfer()
                ),
                robot.autoFastShoot(),
                new Parallel(
                        new WaitUntil(() -> overallTimer.seconds() > 29),
                        new Loop(cycle()),
                        new Infinite(() -> gyroThread.update())
                ),
                robot.drivetrain.followLast(d -> d.velocityCondition(4))
        );
    }

    public ICommand cycle() {
        AtomicInteger selectedPath = new AtomicInteger();
        return new Sequential(
                new Parallel(
                        intake(true),
                        new Instant(() -> selectedPath.set(selectPath())),
                        followCycle(selectedPath)
                ),
                new Parallel(
                        followCycle(selectedPath),
                        new Instant(() -> {
                            robot.feederWheel.start();
                            robot.turret.setPosition(ServoTurret.EIGHTEEN_SECOND_SET);
                        }),
                        transfer()
                ),
                robot.autoFastShoot()
        );
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

    public ICommand intake(boolean stopShooter) {
        return new Sequential(
                new Instant(() -> {
                    if (stopShooter) {
                        robot.flywheel.stop();
                        robot.feederWheel.intakeState();
                    }
                }),
                new Instant(() -> robot.table.setStateCommandless(Table.RelativeState.BALL1)),
                new Wait(725),
                new Parallel(
                        robot.popper.neutral(),
                        robot.intake()
                )
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
}
