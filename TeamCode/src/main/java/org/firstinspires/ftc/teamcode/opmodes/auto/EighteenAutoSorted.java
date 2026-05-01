package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.geometry.Pose;
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

import org.firstinspires.ftc.teamcode.R;
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
    private boolean sort = true;

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
                    if (robot.intakeMotor.atState(IntakeMotor.IntakeState.INTAKE))
                        robot.tableCompartments.intakeThread.update();
                    Pose pose = robot.drivetrain.follower.getPose();
                    if (pose.distanceFrom(new Pose()) > 0.01)
                        Drivetrain.startPose = robot.drivetrain.follower.getPose();
                    telemetry.addData("motif", Globals.randomizationState);
                    telemetry.addData("llouput", robot.limelight.getCurrentPipeline());
                    telemetry.update();
                }),
                new Sequential(
                        new WaitUntil(() -> !opModeInInit()),
                        new Instant(overallTimer::reset),
                        new Instant(() -> {
                            //Preloads=======
                            robot.flywheel.setVelocity(Flywheel.FAR_VELOCITY - 35);
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
                                            gyroThread.setState(TrackState.FAR_ONE);
                                        }),
                                        new Wait(400)
                                ),
                                new Wait(800)
                        ),
                        robot.autoFastShoot(),

                        //Second set=====
                        new Parallel(
                                new Sequential(
                                        new Instant(() -> robot.table.setStateCommandless(Table.RelativeState.BALL1)),
                                        new Wait(600),
                                        robot.popper.neutral()
                                ),
                                robot.intake(),
                                robot.intakeGate.open(),
                                robot.drivetrain.follow(),
                                new Instant(() -> robot.flywheel.setVelocity(Flywheel.FAR_VELOCITY - 45))
                        ),
                        new Wait(200),
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
                                    if (Globals.allianceColor.equals(AllianceColor.Blue))
                                        GyroThread.NEUTRAL_OFFSET += 2/255d;
                                })
                        ),
                        robot.autoFastShoot(),
                        new Wait(200),

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
                                    robot.flywheel.setVelocity(Flywheel.MEDIUM_VELOCITY - 20);
                                    if (Globals.allianceColor.equals(AllianceColor.Blue))
                                        GyroThread.NEUTRAL_OFFSET -= 2/255d;
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
                                    robot.hood.setPosition(Hood.MEDIUM_PRESET - 14 / 255f);
                                    robot.flywheel.setVelocity(Flywheel.MEDIUM_VELOCITY - 10);
                                })
                        ),
                        new Sequential(
                                new Parallel(
                                        robot.drivetrain.follow(),
                                        robot.intakeGate.close(),
                                        new Instant(() -> robot.intakeMotor.outtake())
                                ),
                                new Wait(1300),
                                new Instant(() -> GyroThread.NEUTRAL_OFFSET = 1.5/255d)
                        ),
                        new Parallel(
                                new Instant(() -> GyroThread.NEUTRAL_OFFSET = 0),
                                robot.drivetrain.follow(),
                                shootFromGate(() -> Globals.randomizationState != null)
                        ),
                        robot.autoShoot(() -> !sort),
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
                                robot.drivetrain.followNext(d -> d.tValueCondition(0.9) || d.velocityCondition(5), 2250),
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
                                robot.drivetrain.followNext(d -> d.velocityCondition(4) || d.follower.atParametricEnd(), 3000),
                                transferSorted(),
                                new Instant(() -> {
                                    robot.hood.setPosition(Hood.FAR_PRESET - 5/255d);
                                    if (Globals.allianceColor.equals(AllianceColor.Red))
                                        robot.flywheel.setVelocity(Flywheel.FAR_VELOCITY - 40);
                                    else
                                        robot.flywheel.setVelocity(Flywheel.FAR_VELOCITY - 15);
                                    robot.feederWheel.start();
                                    if (Globals.allianceColor.equals(AllianceColor.Red))
                                        GyroThread.NEUTRAL_OFFSET -= 1/255d;
                                    else
                                        GyroThread.NEUTRAL_OFFSET += 2/255d;
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
        return new Sequential(
                new Deadline(
                    new Wait(150),
                    robot.intakeGate.close(),
                    new Instant(robot.intakeTilt::transfer),
                    new Sequential(
                            new Instant(robot.intakeMotor::outtake),
                            new Wait(50),
                            new Instant(robot.intakeMotor::stop)
                    )
                ),
                new Parallel(
                    robot.popper.pop(),
                    robot.splitter.neutral(),
                    new Instant(robot.feederWheel::start)
                )
        );
    }

    public ICommand transferSorted() {
        return new Sequential(
                new Instant(() -> robot.intakeTilt.transfer()),
                new Parallel(
                        new Sequential(
                                new Wait(500),
                                new Instant(robot.intakeMotor::outtake),
                                new Wait(50),
                                new Instant(robot.intakeMotor::stop)
                        ),
                        new Sequential(
                                robot.intakeGate.close(),
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
                        robot.popper.pop()
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
                    robot.flywheel.setVelocity(Flywheel.NEAR_VELOCITY);
                    sort = isSort.getAsBoolean();
                }),
                robot.intakeGate.close(),
                new Parallel(
                        new Sequential(
                                new Instant(robot.intakeMotor::intake),
                                new Wait(200),
                                new Instant(robot.intakeMotor::stop)
                        ),
                        sort ? new Race(
                                robot.tableCompartments.populateAuto(),
                                new Wait(300)
                        ) : Commands.NOOP
                ),
                sort ? robot.sortAuto() : Commands.NOOP,
                new Instant(() -> robot.feederWheel.start()),
                robot.popper.pop()
        );
    }
}
