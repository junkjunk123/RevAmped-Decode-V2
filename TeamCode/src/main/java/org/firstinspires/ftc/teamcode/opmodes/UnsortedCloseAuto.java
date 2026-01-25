package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Popper;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.commands.channel.Channel;
import org.firstinspires.ftc.teamcode.utils.commands.channel.Channels;

public class UnsortedCloseAuto extends OpModeCommand {
    private Robot robot;
    private final ElapsedTime overallTimer = new ElapsedTime();

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);

        schedule(
                new Infinite(robot::update),
                new Sequential(
                        new WaitUntil(() -> !opModeInInit()),
                        new Instant(overallTimer::reset),
                        new Instant(() -> {
                            robot.flywheel.closeAuto();
                            robot.hood.near();
                            robot.turret.move(Turret.MoveState.CLOSE_AUTO);
                        }),
                        intakePreload(true),
                        shootPreload(true)
                )
        );
    }

    private ICommand intakePreload(boolean isFirst) {
        return new Sequential(
                new Instant(() -> {
                    robot.flywheel.stop();
                    robot.intakeMotor.intake();
                }),
                new Parallel(
                        robot.resetTable(),
                        robot.drivetrain.followNext(d -> d.tValueCondition(0.8) && d.velocityCondition(), getIntakeTimeout())
                )
        );
    }

    private ICommand shootPreload(boolean isFirst) {
        return new Sequential(
                new Parallel(
                        robot.drivetrain.followNext(d -> d.velocityCondition() && d.tValueCondition(0.8), getShootTimeout()),
                        new Conditional(
                                () -> !isFirst,
                                Commands.NOOP,
                                new Sequential(
                                        new Wait(200),
                                        new Instant(() -> robot.intakeMotor.outtakeSlow()),
                                        new Wait(100),
                                        new Instant(() -> {
                                            robot.intakeMotor.intake();
                                        })
                                )
                        ),
                        new Instant(() -> robot.flywheel.closeAuto()),
                        new Sequential(
                                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.75)),
                                new Instant(robot.popper::pop)
                        )
                ),
                robot.shootAll()
        );
    }

    private ICommand intakeAndShoot(int iteration) {
        return new Sequential(
                robot.drivetrain.followNext(f -> f.velocityCondition(4), 3000),
                new Parallel(
                        robot.popper.pop(),
                        robot.shootAll(),
                        new Parallel(
                                robot.resetTable(),
                                robot.drivetrain.followNext(f -> f.velocityCondition(4),
                                        getIntakeTimeout())
                        )
                ),
                waitForIntake(iteration)
        );
    }

    @Override
    public void end() {
        Drivetrain.startPose = robot.drivetrain.follower.getPose();
        //Turret.startPos = robot.turret.getTargetPosition();
    }

    private ICommand waitForIntake(int iteration) {
        if (iteration == 0) return Commands.NOOP;
        return new Wait(0);
    }

    public int getIntakeTimeout() {
        return 3000;
    }

    public int getShootTimeout() {
        return 3000;
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
