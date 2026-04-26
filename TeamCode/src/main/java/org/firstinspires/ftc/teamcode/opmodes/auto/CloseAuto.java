package org.firstinspires.ftc.teamcode.opmodes.auto;

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
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.opmodes.paths.UnsortedCloseAutoPaths;

public class CloseAuto extends OpModeCommand {
    private Robot robot;
    private final ElapsedTime overallTimer = new ElapsedTime();

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, new UnsortedCloseAutoPaths());
        robot.turret.setPosition(ServoTurret.UNSORTED_AUTO_PRELOADS.getPos());
        robot.hood.unsortedAuto();
        robot.table.setStateCommandless(Table.RelativeState.BALL0);

        schedule(
                new Infinite(() -> {
                    robot.update();
                    Drivetrain.startPose = robot.drivetrain.follower.getPose();
                    if (robot.intakeMotor.atState(IntakeMotor.IntakeState.INTAKE))
                        robot.tableCompartments.intakeThread.update();
                }),
                new Sequential(
                        new WaitUntil(() -> !opModeInInit()),
                        new Instant(() -> {
                            robot.flywheel.setVelocity(Flywheel.UNSORTED_AUTO_VELOCITY - 35);
                            robot.feederWheel.start();
                            robot.splitter.neutral();
                            robot.intakeGate.close();
                            overallTimer.reset();
                        }),
                        shootFirstThree(),
                        new Parallel(
                                intake(1),
                                robot.drivetrain.follow(),
                                robot.drivetrain.follow()
                        ),
                        new Parallel(
                                robot.drivetrain.follow(),
                                transfer(),
                                new Instant(() -> {robot.flywheel.setVelocity(Flywheel.MEDIUM_VELOCITY - 65); robot.hood.near();})
                        ),
                        robot.autoFastShoot(),
                        gateCycle(2),
                        gateCycle(3),
                        gateCycle(4),
                        gateCycle(5),
                        new Parallel(
                                intake(6),
                                new Sequential(
                                        new Wait(500),
                                        robot.drivetrain.follow()
                                )
                        ),
                        new Parallel(
                                robot.drivetrain.follow(),
                                transfer(),
                                new Instant(() -> robot.flywheel.setVelocity(Flywheel.NEAR_VELOCITY - 30))
                        ),
                        robot.autoFastShoot(),
                        new Instant(() -> {robot.table.setStateCommandless(Table.RelativeState.BALL1); robot.intakeMotor.stop();})
                )
        );
    }

    public ICommand gateCycle(int i) {
        return new Sequential(
                new Race(
                        new WaitUntil(() -> robot.tableCompartments.intakeThread.hasThree),
                        new Parallel(
                                intakeFromGate(i),
                                robot.drivetrain.follow()
                        )
                ),
                new Parallel(
                        robot.drivetrain.follow(),
                        shootFromGate()
                )
        );
    }

    public ICommand shootFirstThree() {
        return new Deadline(
                new Sequential(
                        robot.popper.pop(),
                        new WaitUntil(() -> robot.drivetrain.tValueCondition(0.4)),
                        new Parallel(
                                robot.autoFastShoot(),
                                new Sequential(
                                        new Wait(150),
                                        new Instant(() -> {robot.turret.setPosition(
                                                robot.turret.getPosition() +
                                                        4/255f * (int) Math.signum(ServoTurret.REST - robot.turret.getPosition())
                                        ); robot.flywheel.setVelocity(Flywheel.UNSORTED_AUTO_VELOCITY + 90);})
                                )
                        )
                ),
                robot.drivetrain.followNext(d -> d.tValueCondition(0.9), 3000)
        );
    }

    public ICommand intake(int i) {
        return new Sequential(
                resetShooter(),
                new Instant(() -> {
                    if (i == 6) robot.table.setStateCommandless(Table.RelativeState.BALL2);
                    else if (i == 1) robot.table.setStateCommandless(Table.RelativeState.BALL1);
                    else robot.table.setStateCommandless(Table.RelativeState.BALL0);
                    robot.intakeMotor.intake();
                    robot.intakeGate.open();
                }),
                new Wait(300),
                new Parallel(
                        new Instant(() -> aimTurret(i)),
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
                new Race(
                        new Wait(600),
                        new WaitUntil(() -> robot.tableCompartments.intakeThread.hasThree)
                ),
                new Instant(() -> {
                    robot.intakeTilt.transfer();
                    robot.intakeMotor.outtake();
                }),
                new Parallel(
                        robot.popper.pop(),
                        new Instant(() -> robot.feederWheel.start()),
                        robot.splitter.neutral(),
                        robot.intakeGate.close()
                )
        );
    }

    public ICommand resetShooter() {
        return new Instant(() -> {
            robot.flywheel.stop();
            robot.feederWheel.stop();
        });
    }

    public ICommand intakeFromGate(int i) {
        return new Sequential(
                resetShooter(),
                new Instant(() -> robot.table.setStateCommandless(Table.RelativeState.BALL0)),
                new Wait(300),
                new Parallel(
                        new Instant(() -> aimTurret(i)),
                        new Sequential(
                                new Wait(300),
                                robot.popper.neutral()
                        ),
                        new Sequential(
                                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.75)),
                                robot.intake()
                        )
                ),
                new Race(
                        new Wait(1400),
                        new WaitUntil(() -> robot.tableCompartments.intakeThread.hasThree)
                )
        );
    }

    public ICommand shootFromGate() {
        return new Sequential(
                new Instant(() -> {
                    robot.intakeTilt.transfer();
                    robot.intakeMotor.outtake();
                    robot.flywheel.setVelocity(Flywheel.MEDIUM_VELOCITY - 65);
                }),
                new Parallel(
                        robot.splitter.neutral(),
                        robot.intakeGate.close(),
                        robot.popper.pop()
                ),
                new Sequential(
                        new Instant(robot.intakeMotor::intake),
                        new Wait(200),
                        new Instant(robot.intakeMotor::stop)
                ),
                new Instant(() -> robot.feederWheel.start()),
                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.9)),
                robot.autoFastShoot()
        );
    }

    public void aimTurret(int i) {
        switch (i) {
            case 1 -> robot.turret.setPosition(ServoTurret.UNSORTED_SET_1.getPos());
            case 2 -> robot.turret.setPosition(ServoTurret.UNSORTED_SET_2.getPos());
            case 3 -> robot.turret.setPosition(ServoTurret.UNSORTED_SET_3.getPos());
            case 4 -> robot.turret.setPosition(ServoTurret.UNSORTED_SET_4.getPos());
            case 5 -> robot.turret.setPosition(ServoTurret.UNSORTED_SET_5.getPos());
            case 6 -> robot.turret.setPosition(ServoTurret.UNSORTED_FINAL.getPos());
        }
    }
}