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
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeTilt;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.GyroThread;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.opmodes.paths.EighteenPathsBlue;
import org.firstinspires.ftc.teamcode.opmodes.paths.EighteenPathsRed;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.commands.ArtifactColor;
import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Lazy;
import org.firstinspires.ftc.teamcode.utils.commands.RandomizationState;
import org.firstinspires.ftc.teamcode.utils.math.projectile.TrackState;

import java.util.function.BooleanSupplier;

public class EighteenAutoSorted extends OpModeCommand {
    private Robot robot;
    private final ElapsedTime overallTimer = new ElapsedTime();
    private GyroThread gyroThread;
    private static boolean testSlowShoot = false;
    private boolean useGyro = false;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, Globals.allianceColor.equals(AllianceColor.Red) ? new EighteenPathsRed() : new EighteenPathsBlue());
        robot.tableCompartments.populate(ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE);
        robot.turret.setPosition(ServoTurret.EIGHTEEN_DETECTION.getPos());
        gyroThread = new GyroThread(robot);

        schedule(
                new Infinite(() -> {
                    robot.update();
                    gyroThread.update(useGyro);
                    if (robot.intakeMotor.atState(IntakeMotor.IntakeState.INTAKE)) robot.tableCompartments.intakeThread.update();
                    Pose pose = robot.drivetrain.follower.getPose();
                    if (pose.distanceFrom(new Pose()) > 0.01) Drivetrain.startPose = robot.drivetrain.follower.getPose();
                    telemetry.addData("motif", Globals.randomizationState);
                    telemetry.addData("llouput",robot.limelight.getCurrentPipeline());
                    telemetry.update();
                }),
                new Sequential(
                        new WaitUntil(() -> !opModeInInit()),
                        new Instant(overallTimer::reset),
                        new Instant(() -> {
                            //Preloads=======
                            robot.flywheel.setVelocity(Flywheel.FAR_VELOCITY + 30);
                            robot.hood.far();
                            robot.feederWheel.start();
                            robot.popper.popCommandless();
                        }),
                        new Parallel(
                                new Sequential(
                                        !testSlowShoot ? new Race(
                                                robot.limelight.detectMotif(),
                                                new Wait(3000)
                                        ) : new Instant(() -> Globals.randomizationState = RandomizationState.GPP),
//                                        new Instant(() -> robot.turret.setPosition(ServoTurret.EIGHTEEN_PRELOADS.getPos())),
                                        new Instant(() -> {
                                            useGyro = true;
                                            gyroThread.setState(TrackState.FAR_ONE);}),
                                        new Wait(400)
                                ),
                                new Wait(800)
                        ),
                        robot.autoFastShoot(),
                        new Parallel(
                                robot.intake(),
                                robot.intakeGate.open()
                        ),

                        //Second set=====
                        new Parallel(
                                intake(false),
                                robot.turret.resetTurret(),
                                robot.drivetrain.follow(),
                                new Instant(() -> robot.flywheel.setVelocity(Flywheel.FAR_VELOCITY + 25))
                        ),
                        new Wait(250),
                        new Parallel(
                                robot.drivetrain.follow(),
                                new Sequential(
                                        new Wait(600),
                                        transfer()
                                ),
                                new Instant(() -> {
//                                    robot.turret.setPosition(ServoTurret.EIGHTEEN_FIRST_SET.getPos());
                                    gyroThread.setState(TrackState.FAR_TWO);
                                    robot.hood.far();
                                })
                        ),
                        robot.autoFastShoot(),

                        //Third set=======
                        new Parallel(
                                intake(true),
                                robot.turret.resetTurret(),
                                robot.drivetrain.follow()
                        ),
                        new Parallel(
                                robot.drivetrain.follow(),
                                new Instant(() -> {
                                    robot.hood.medium();
                                    robot.flywheel.setVelocity(Flywheel.MEDIUM_VELOCITY - 10);
//                                    GyroThread.NEUTRAL_OFFSET = ??
                                    gyroThread.setState(TrackState.CLOSE_ONE);
                                    useGyro = true;
                                    robot.turret.setPosition(ServoTurret.EIGHTEEN_SECOND_SET.getPos());
                                }),
                                new Sequential(
                                        new WaitUntil(() -> robot.drivetrain.tValueCondition(0.5d)),
                                        transfer()
                                )
                        ),
                        robot.autoFastShoot(IntakeTilt.TiltState.GATE_INTAKE),

                        //Fourth Set=======
                        new Parallel(
                                intakeFromGate(),
                                robot.turret.resetTurret(),
                                robot.drivetrain.follow(),
                                new Instant(() -> {
                                    robot.hood.setPosition(Hood.MEDIUM_PRESET - 14/255f);
                                    robot.flywheel.setVelocity(Flywheel.MEDIUM_VELOCITY+20);
                                })
                        ),
                        new Sequential(
                                new Parallel(
                                    robot.drivetrain.follow(),
                                    robot.intakeGate.close(),
                                    new Instant(() -> robot.intakeMotor.outtake())
                                ),
                                new Wait(1500)
//                                new Instant(() -> robot.turret.setPosition(ServoTurret.EIGHTEEN_GATE_SHOOT.getPos()))
                        ),
                        new Parallel(
                                robot.drivetrain.follow(),
                                shootFromGate(() -> Globals.randomizationState != null)
                        ),
                        //Fifth set=======
                        new Parallel(
                                intake(true),
                                robot.turret.resetTurret(),
                                new Sequential(
                                        new Wait(500),
                                        robot.drivetrain.follow()
                                )
                        ),
                        new Parallel(
                                robot.drivetrain.follow(),
                                transferSorted(),
                                new Instant(() -> robot.flywheel.setVelocity(Flywheel.NEAR_VELOCITY + 10))
//                                new Instant(() -> robot.turret.setPosition(ServoTurret.EIGHTEEN_THIRD_SET.getPos()))
                        ),
                        robot.autoShoot(() -> Globals.randomizationState == null),

                        //Sixth set========
                        new Parallel(
                                intake(true),
                                robot.turret.resetTurret(),
                                new Race(
                                    new Parallel(
                                        robot.drivetrain.follow(),
                                        new Wait(300)
                                        ),
                                    new WaitUntil(() -> robot.tableCompartments.intakeThread.hasThree)
                                )
                        ),
                        new Parallel(
//                                new Instant(() -> robot.turret.setPosition(ServoTurret.EIGHTEEN_FIFTH_SET.getPos())),
                                new Instant(() -> gyroThread.setState(TrackState.FAR_ONE)),
                                robot.drivetrain.follow(),
                                transferSorted(),
                                new Instant(() -> {
                                    robot.hood.far();
                                    robot.flywheel.setVelocity(Flywheel.FAR_VELOCITY);
                                    robot.feederWheel.start();
                                })
                        ),
                        robot.autoShoot(() -> Globals.randomizationState == null),
                        new Parallel(
                                new Sequential(
                                        new Instant(() -> robot.table.setStateCommandless(Table.RelativeState.BALL1)),
                                        new Wait(600),
                                        new Instant(() -> robot.popper.popCommandless())
                                ),
                                robot.drivetrain.follow()
                        )
                )
        );
    }

    public ICommand intake(boolean stopShooter) {
        return new Sequential(
                new Instant(() -> {
                    if (stopShooter) {
                        robot.flywheel.stop();
                        robot.feederWheel.stop();
                    }
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
        return new Parallel(
                new Instant(robot.intakeTilt::transfer),
                new Wait(200),
                new Instant(robot.feederWheel::start),
                new Sequential(
                        new Wait(200),
                        new Instant(robot.intakeMotor::outtake)
                ),
                robot.popper.pop(),
                robot.splitter.neutral(),
                robot.intakeGate.close()
        );
    }

    public ICommand transferSorted() {
        return new Sequential(
                new Instant(() -> robot.intakeTilt.transfer()),
                new Parallel(
                        new Sequential(
                                new Wait(500),
                                new Instant(robot.intakeMotor::outtake)
                        ),
                        new Sequential(
                                new Parallel(
                                        robot.splitter.neutral(),
                                        robot.intakeGate.close()
                                ),
                                new Lazy(() -> {
                                    if (Globals.randomizationState != null) {
                                        return new Sequential(
                                                new Race(
                                                        robot.tableCompartments.populateAuto(),
                                                        new Wait(300)
                                                ),
                                                robot.sortAuto()
                                        );
                                    }
                                    return Commands.NOOP;
                                })
                        )
                ),
                new Parallel(
                        new Instant(() -> robot.feederWheel.start()),
                        robot.popper.pop(),
                        new Sequential(
                                new Wait(250),
                                new Instant(robot.intakeMotor::stop)
                        )
                )
        );
    }

    public ICommand intakeFromGate() {
        return new Sequential(
                new Instant(() -> {
                    robot.flywheel.stop();
                    robot.feederWheel.stop();
                }),
                new Instant(() -> robot.table.setStateCommandless(Table.RelativeState.BALL1)),
                new Wait(300),
                new Parallel(
                        new Sequential(
                                new Wait(300),
                                robot.popper.neutral()
                        ),
                        new Sequential(
                                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.75)),
                                robot.gateIntake()
                        )
                ),
                new Race(
                    new WaitUntil(() -> robot.tableCompartments.intakeThread.hasThree),
                    new Wait(1600)
                )
        );
    }

    public ICommand shootFromGate(BooleanSupplier isSort) {
        return new Sequential(
                new Instant(() -> {
                    robot.intakeTilt.transfer();
                    robot.intakeMotor.outtake();
                    robot.flywheel.setVelocity(Flywheel.NEAR_VELOCITY - 20);
                }),
                new Parallel(
                        robot.splitter.neutral(),
                        robot.intakeGate.close()
                ),
                new Parallel(
                        new Sequential(
                                new Instant(robot.intakeMotor::intake),
                                new Wait(200),
                                new Instant(robot.intakeMotor::stop)
                        ),
                        isSort.getAsBoolean() ? new Race(
                                robot.tableCompartments.populateAuto(),
                                new Wait(300)
                        ) : Commands.NOOP
                ),
                isSort.getAsBoolean() ? robot.sortAuto() : Commands.NOOP,
                new Instant(() -> robot.feederWheel.start()),
                robot.popper.pop(),
                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.8)),
                robot.autoShoot(() -> !isSort.getAsBoolean())
        );
    }
}
