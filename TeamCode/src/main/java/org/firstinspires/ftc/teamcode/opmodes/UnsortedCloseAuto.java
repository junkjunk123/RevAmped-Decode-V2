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
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.opmodes.paths.UnsortedCloseAutoPaths;
import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;

public class UnsortedCloseAuto extends OpModeCommand {
    private Robot robot;
    private final ElapsedTime overallTimer = new ElapsedTime();

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, new UnsortedCloseAutoPaths());
        robot.popper.popCommandless();
        robot.hood.medium();
        robot.table.setUseEncoder(false);

        schedule(
                new Infinite(robot::update),
                new Sequential(
                        new WaitUntil(() -> !opModeInInit()),
                        new Instant(overallTimer::reset),
                        new Instant(() -> {
                            robot.flywheel.unsortedAuto();
                            robot.turret.setTargetPosition(Turret.UNSORTED_AUTO_PRELOADS);
                        }),
                        new Parallel(
                                robot.drivetrain.followNext(d -> d.velocityCondition(4), 3000)
                        ),
                        robot.autoFastShoot(),
                        intakePreload(true),
                        shootPreload(true),
                        intakeFromGate(0),
                        shootFromGate(0),
                        intakeFromGate(1),
                        shootFromGate(1),
                        intakeFromGate(2),
                        shootFromGate(2),
                        //intakeFromGate(3),
                        //shootFromGate(3),
                        intakePreload(false),
                        shootLastPreload(true),
                        park()
                )
        );
    }

    private ICommand intakeFromGate(int iteration) {
        return new Sequential(
                new Instant(() -> robot.flywheel.stop()),
                new Instant(() -> robot.intakeMotor.stop()),
                new Parallel(
                        new Sequential(
                                resetTable(),
                                robot.popper.block()
                        ),
                        robot.drivetrain.followNext(d -> d.tValueCondition(0.8) && d.velocityCondition(), getIntakeTimeout()),
                        new Sequential(
                                new Wait(400),
                                new Instant(() -> robot.intakeMotor.intake())
                        )
                ),
                new Wait(1500)
        );
    }

    private ICommand resetTable() {
        return new Sequential(
                //new Instant(robot.intakeMotor::stop),
                robot.popper.neutral(),
                new Instant(() -> robot.table.setPosition(Table.BALL1)),
                new Wait(650),
                new Instant(robot.intakeMotor::intake)
        );
    }

    private ICommand shootFromGate(int iteration) {
        return new Sequential(
                new Instant(() -> {
                    robot.flywheel.unsortedAuto();
                    robot.hood.medium();
                    robot.turret.setTargetPosition(Turret.UNSORTED_GATE);
                }),
                new Parallel(
                        robot.drivetrain.followNext(d -> d.velocityCondition() && d.tValueCondition(0.8), getShootTimeout()),
                        new Sequential(
                                robot.popper.neutral(),
                                new Instant(() -> robot.intakeMotor.intake()),
                                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.6)),
                                robot.popper.pop()
                        ),
                        new Sequential(
                                new Wait(250),
                                new Instant(robot.intakeMotor::stop)
                        )
                ),
                robot.autoFastShoot()
        );
    }

    private ICommand intakePreload(boolean isFirst) {
        return new Sequential(
                new Instant(() -> {
                    robot.flywheel.stop();
                    if (isFirst) robot.turret.setTargetPosition(Turret.UNSORTED_GATE);
                    else robot.turret.setTargetPosition(Turret.UNSORTED_FINAL);
                    robot.intakeMotor.intake();
                }),
                new Parallel(
                        resetTable(),
                        new Sequential(
                                isFirst ? Commands.NOOP : new Wait(600),
                                robot.drivetrain.followNext(d -> d.tValueCondition(0.8) && d.velocityCondition(), getIntakeTimeout())
                        ),
                        isFirst ? new Instant(() -> robot.flywheel.unsortedAuto()) : Commands.NOOP
                )
        );
    }

    private ICommand shootPreload(boolean isFirst) {
        return new Sequential(
                new Parallel(
                        robot.drivetrain.followNext(d -> d.velocityCondition() && d.tValueCondition(0.8), getShootTimeout()),
                        isFirst ? Commands.NOOP : new Sequential(
                                new Wait(200),
                                new Instant(() -> robot.intakeMotor.outtakeSlow()),
                                new Wait(100),
                                new Instant(() -> robot.intakeMotor.intake())
                        ),
                        new Instant(() -> {
                            robot.flywheel.unsortedAuto();
                            robot.hood.medium();
                        }),
                        new Sequential(
                                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.75)),
                                robot.popper.pop()
                        )
                ),
                robot.autoFastShoot()
        );
    }

    private ICommand shootLastPreload(boolean isFirst) {
        return new Sequential(
                new Parallel(
                        robot.drivetrain.followNext(d -> d.velocityCondition() && d.tValueCondition(0.8), getShootTimeout()),
                        isFirst ? Commands.NOOP : new Sequential(
                                new Wait(200),
                                new Instant(() -> robot.intakeMotor.outtakeSlow()),
                                new Wait(100),
                                new Instant(() -> robot.intakeMotor.intake())
                        ),
                        new Instant(() -> {
                            robot.flywheel.setVelocity(660);
                            robot.hood.near();
                        }),
                        new Sequential(
                                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.75)),
                                robot.popper.pop()
                        )
                ),
                robot.autoFastShoot()
        );
    }

    @Override
    public void end() {
        Drivetrain.startPose = robot.drivetrain.follower.getPose();
        //Turret.startPos = robot.turret.getTargetPosition();
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

    @Override
    public void execute() {
        telemetry.addData("tableVel", robot.table.getEncoder().getVelocity());
        telemetry.update();
    }
}
