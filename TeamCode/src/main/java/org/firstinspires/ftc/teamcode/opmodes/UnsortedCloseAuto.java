package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Deadline;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Popper;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
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
        robot.turret.setTargetPosition(Turret.UNSORTED_AUTO_PRELOADS);
        robot.popper.popCommandless();
        robot.table.setStateCommandless(Table.RelativeState.BALL0);
        robot.hood.unsortedAuto();

        schedule(
                new Infinite(() -> {
                    robot.update();
                    Drivetrain.startPose = robot.drivetrain.follower.getPose();
                    telemetry.addData("turret target", robot.turret.getTargetPosition());
                    telemetry.addData("turret encoder", robot.turret.getPosition());
                    telemetry.update();
                }),
                new Sequential(
                        new WaitUntil(() -> !opModeInInit()),
                        new Instant(overallTimer::reset),
                        new Instant(() -> robot.flywheel.setVelocity(Flywheel.UNSORTED_AUTO_VELOCITY + 60)),
                        new Deadline(
                                new Sequential(
                                        new WaitUntil(() -> robot.drivetrain.tValueCondition(0.4)),
                                        new Parallel(
                                                robot.autoFastShoot(),
                                                new Sequential(
                                                        new Wait(50),
                                                        new Instant(() -> {robot.turret.setTargetPosition(
                                                                robot.turret.getTargetPosition() -
                                                                        20 * (int) Math.signum(robot.turret.getTargetPosition())
                                                        ); robot.flywheel.setVelocity(Flywheel.UNSORTED_AUTO_VELOCITY + 95);})
                                                )
                                        )
                                ),
                                robot.drivetrain.followNext(d -> d.velocityCondition(4), 3000)
                        ),
                        intakePreload(true),
                        shootPreload(true),
                        intakeFromGate(0),
                        shootFromGate(0),
                        intakeFromGate(1),
                        shootFromGate(1),
                        intakeFromGate(2),
                        shootFromGate(2),
                        intakeFromGate(3),
                        shootFromGate(3),
                        intakePreload(false),
                        shootPreload(false),
                        park()
                )
        );
    }

    private ICommand intakeFromGate(int iteration) {
        return new Sequential(
                new Instant(() -> {
                    robot.flywheel.stop();
                    robot.intakeMotor.stop();
                }),
                new Parallel(
                        resetTableBlock(),
                        new Sequential(
                                robot.drivetrain.followNext(d -> d.tValueCondition(0.8) && d.velocityCondition(), getIntakeTimeout()),
                                new Instant(() -> {
                                    robot.turret.unsortedAutoSet(iteration + 1);
                                    robot.flywheel.unsortedAuto();
                                })
                        ),
                        new Sequential(
                                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.75)),
                                new Instant(() -> {
                                    robot.intakeMotor.intakeGate();
                                    robot.turret.setTargetPosition(robot.turret.getTargetPosition() -
                                            250 * (int) Math.signum(robot.turret.getTargetPosition()));
                                })
                        )
                ),
                new Wait(1200)
        );
    }

    private ICommand resetTable() {
        return new Sequential(
                //new Instant(robot.intakeMotor::stop),
                robot.popper.neutral(),
                new Instant(() -> robot.table.setStateCommandless(Table.RelativeState.BALL0)),
                new Wait(650)
        );
    }

    private ICommand resetTableBlock() {
        return new Sequential(
                //new Instant(robot.intakeMotor::stop),
                new Instant(() -> robot.table.setStateCommandless(Table.RelativeState.BALL0)),
                new Wait(650),
                robot.popper.blockFromPop()
        );
    }


    private ICommand shootFromGate(int iteration) {
        return new Parallel(
                robot.drivetrain.followNext(d -> d.velocityCondition() && d.tValueCondition(0.8), getShootTimeout()),
                new Sequential(
                        new Instant(() -> robot.intakeMotor.outtakeSlow()),
                        new Sequential(
                                new Parallel(
                                        robot.popper.neutral(),
                                        new Sequential(
                                                new Wait(175),
                                                new Instant(robot.intakeMotor::intake),
                                                new Wait(300)
                                        )
                                ),
                                robot.popper.pop(),
                                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.80)),
                                robot.autoFastShoot()
                        )
                )
        );
    }

    private ICommand intakePreload(boolean isFirst) {
        return new Sequential(
                new Instant(() -> {
                    if (!isFirst) {
                        robot.turret.setTargetPosition(Turret.UNSORTED_FINAL);
                        robot.flywheel.setVelocity(Flywheel.NEAR_VELOCITY - 10);
                    }

                    robot.intakeMotor.intake();
                }),
                new Parallel(
                        resetTable(),
                        new Sequential(
                                new Wait(600),
                                robot.drivetrain.followNext(d -> d.tValueCondition(0.8) &&
                                        d.velocityCondition(), getIntakeTimeout()),
                                isFirst ? new Instant(() -> robot.flywheel.unsortedAuto()) : Commands.NOOP,
                                new Sequential(
                                        new WaitUntil(() -> robot.drivetrain.tValueCondition(0.5)),
                                        new Instant(() -> robot.turret.unsortedAutoSet(0))
                                )
                        )
                )
        );
    }

    private ICommand shootPreload(boolean isFirst) {
        return new Sequential(
                new Instant(() -> {
                    if (isFirst) {
                        robot.flywheel.unsortedAuto();
                        robot.hood.unsortedAuto();
                    } else {
                        robot.flywheel.setVelocity(Flywheel.NEAR_VELOCITY - 30);
                        robot.hood.setPosition(Hood.NEAR_PRESET - 10/255f);
                    }
                }),
                isFirst ? new Parallel(
                        robot.drivetrain.followNext(d -> d.velocityCondition() && d.tValueCondition(0.8), getShootTimeout()),
                        preloadShootArmActions(true)
                ) : new Deadline(
                        preloadShootArmActions(false),
                        robot.drivetrain.followNext(d -> d.velocityCondition() && d.tValueCondition(0.8), getShootTimeout())
                )
        );
    }

    private ICommand preloadShootArmActions(boolean isFirst) {
        return isFirst ? new Sequential(
                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.5)),
                robot.popper.pop(),
                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.80)),
                robot.autoFastShoot()
        ) : new Sequential(
                new Wait(200),
                new Instant(() -> robot.intakeMotor.outtakeSlow()),
                new Wait(100),
                new Instant(() -> robot.intakeMotor.intake()),
                new Wait(150),
                robot.popper.pop(),
                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.80)),
                new Wait(300),
                robot.autoFastShoot()
        );
    }

    @Override
    public void end() {
        //Drivetrain.startPose = robot.drivetrain.follower.getPose();
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
    public void execute() {
        telemetry.update();
    }

    @Override
    public void initializeLoop() {
        robot.update();
    }
}
