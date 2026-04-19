package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Deadline;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.opmodes.paths.UnsortedCloseAutoPaths;

public class CloseAuto extends OpModeCommand {
    private Robot robot;
    private final ElapsedTime overallTimer = new ElapsedTime();

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, new UnsortedCloseAutoPaths());
        robot.turret.setPosition(ServoTurret.UNSORTED_AUTO_PRELOADS);
        robot.hood.unsortedAuto();

        schedule(
                new Infinite(() -> {
                    robot.update();
                    Drivetrain.startPose = robot.drivetrain.follower.getPose();
                }),
                new Sequential(
                        new WaitUntil(() -> !opModeInInit()),
                        new Instant(() -> {
                            robot.flywheel.setVelocity(Flywheel.UNSORTED_AUTO_VELOCITY + 60);
                            robot.feederWheel.start();
                            overallTimer.reset();
                        }),
                        shootFirstThree(),
                        new Parallel(
                                intake(1),
                                robot.drivetrain.follow()
                        ),
                        new Wait(250),
                        new Parallel(
                                robot.drivetrain.follow(),
                                transfer(),
                                new Instant(() -> robot.flywheel.medium())
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
                                ),
                                new Instant(() -> robot.turret.setPosition(ServoTurret.UNSORTED_FINAL))
                        ),
                        new Instant(() -> robot.table.setStateCommandless(Table.RelativeState.BALL1)),
                        new Wait(600),
                        new Instant(() -> robot.popper.popCommandless())
                )
        );
    }

    public ICommand gateCycle(int i) {
        return new Sequential(
                new Parallel(
                        intakeFromGate(i),
                        robot.drivetrain.follow()
                ),
                new Parallel(
                        robot.drivetrain.follow(),
                        shootFromGate()
                )
        );
    }

    public ICommand shootFirstThree() {
        return new Deadline(
                robot.drivetrain.followNext(d -> d.tValueCondition(0.9), 3000),
                new Sequential(
                        new WaitUntil(() -> robot.drivetrain.tValueCondition(0.4)),
                        new Parallel(
                                robot.autoFastShoot(),
                                new Sequential(
                                        new Wait(50),
                                        new Instant(() -> {robot.turret.setPosition(
                                                robot.turret.getPosition() -
                                                        20/255f * (int) Math.signum(robot.turret.getPosition())
                                        ); robot.flywheel.setVelocity(Flywheel.UNSORTED_AUTO_VELOCITY + 115);})
                                )
                        )
                )
        );
    }

    public ICommand intake(int i) {
        return new Sequential(
                resetShooter(i),
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
                new Wait(600),
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

    public ICommand resetShooter(int i) {
        return new Instant(() -> {
            robot.flywheel.stop();
            robot.feederWheel.stop();

            switch (i) {
                case 1: robot.turret.setPosition(ServoTurret.UNSORTED_SET_1);
                case 2: robot.turret.setPosition(ServoTurret.UNSORTED_SET_2);
                case 3: robot.turret.setPosition(ServoTurret.UNSORTED_SET_3);
                case 4: robot.turret.setPosition(ServoTurret.UNSORTED_SET_4);
                case 5: robot.turret.setPosition(ServoTurret.UNSORTED_SET_5);
                case 6: robot.turret.setPosition(ServoTurret.EIGHTEEN_THIRD_SET);
            }
        });
    }

    public ICommand intakeFromGate(int i) {
        return new Sequential(
                resetShooter(i),
                new Instant(() -> {
                    robot.turret.setPosition(ServoTurret.EIGHTEEN_GATE_SHOOT);
                    robot.table.setStateCommandless(Table.RelativeState.BALL1);
                }),
                new Wait(300),
                new Parallel(
                        new Sequential(
                                new Wait(300),
                                robot.popper.neutral()
                        ),
                        new Sequential(
                                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.75)),
                                robot.intake()
                        )
                ),
                new Wait(1100)
                /*
                new Race(
                        new Wait(1200),
                        new WaitUntil(() -> robot.frontDistance.hasArtifact())
                )
                 */
        );
    }

    public ICommand shootFromGate() {
        return new Sequential(
                new Instant(() -> {
                    robot.intakeTilt.transfer();
                    robot.intakeMotor.outtake();
                    robot.flywheel.setVelocity(Flywheel.MEDIUM_VELOCITY - 40);
                }),
                new Parallel(
                        robot.splitter.neutral(),
                        robot.intakeGate.close()
                ),
                new Sequential(
                        new Instant(robot.intakeMotor::intake),
                        new Wait(200),
                        new Instant(robot.intakeMotor::stop)
                ),
                new Instant(() -> robot.feederWheel.start()),
                robot.popper.pop(),
                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.8)),
                robot.autoFastShoot()
        );
    }
}
