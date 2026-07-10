package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.SoloAuto.GATE_WAIT;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Deadline;
import com.pedropathing.ivy.groups.Loop;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeDistanceSensors;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurretState;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.TrackingThread;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.opmodes.paths.WCISoloPaths;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.data.TurretCalibration;
import org.firstinspires.ftc.teamcode.utils.math.projectile.SimpleShooterMath;

public class SoloCloseAuto extends OpModeCommand {
    private Robot robot;
    private final ElapsedTime matchTimer = new ElapsedTime();
    private TrackingThread autoTrack;

    public static double PARTNER_LENGTH = 27;
    public static TurretCalibration TURRET_PRESET = TurretCalibration.fromRed(51/255f);

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, new WCISoloPaths(PARTNER_LENGTH));
        autoTrack = new TrackingThread(robot);
        robot.turret.closeSideSpikePreloads();
        robot.hood.setPosition(0.1);

        IntakeDistanceSensors.useSensors = true;
        TrackingThread.velocityCompensation = false;
        TrackingThread.trackHood = false;
        TrackingThread.trackTurret = false;

        robot.gate.setGateOpen();

        schedule(
                new Infinite(() -> {
                    robot.update();
                    Drivetrain.startPose = robot.drivetrain.follower.getPose();
                    autoTrack.update();
                    telemetry.update();
                }),
                new Sequential(
                        new WaitUntil(() -> !opModeInInit()),
                        new Instant(() -> {
                            matchTimer.reset();
                            robot.flywheel.setVelocity(Flywheel.CLOSE_PRELOADS_VEL);
                        }),
                        robot.drivetrain.follow(),
                        robot.autoShoot(),
                        new Instant(() -> {
                            TrackingThread.trackTurret = true;
                            TrackingThread.trackHood = true;
                        }),
                        sideSpikeCycle(),
                        spikeCycle(),
                        gateIntake(true),
                        gateReturn(),
                        thirdSpike(),
                        new Parallel(
                                robot.gate.open(),
                                new Wait(200)
                        ),
                        gateIntake(false),
                        gateReturn(),
                        gateIntake(false),
                        gateReturn(),
                        finalCornerCycle(),
                        partnerIntake(),
                        robot.drivetrain.followLast(Drivetrain.isDone),
                        new Instant(() -> {
                            robot.feederWheel.stop();
                            robot.intake.stopIntake();
                            robot.flywheel.stop();
                        })
                )
        );
    }

    public ICommand sideSpikeCycle() {
        return new Sequential(
                //Intaking
                new Race(
                        robot.drivetrain.follow(),
                        intake()
                ),
                //Shooting
                new Parallel(
                        new Deadline(
                                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.9)),
                                robot.drivetrain.follow() //shooting path
                        ),
                        new Sequential(
                                new Wait(200),
                                new Instant(() -> robot.intake.stopIntake())
                        ),
                        new Sequential(
                                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.5)),
                                robot.gate.open()
                        )
                )
        );
    }

    public ICommand intake() {
        return new Sequential(
                new Instant(() -> SimpleShooterMath.hoodCompOffset = 0),
                robot.resetShooter(),
                new Instant(robot::intake),
                //clear the states at 50% of path to remove any false positives from the previous shoot
                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.5)),
                new Instant(() -> robot.intake.distanceSensors.clear()),
                new Parallel(
                        new Sequential(
                                new WaitUntil(() -> robot.intake.ballInTransfer()),
                                new Instant(robot::stopFeeder)
                        ),
                        new WaitUntil(() -> robot.intake.hasThree())
                )
        );
    }

    public ICommand spikeCycle() {
        return new Parallel(
                robot.drivetrain.follow(),
                new Sequential(
                        new Race(
                                new Sequential(
                                        new Instant(() -> {
                                            if (Globals.allianceColor == AllianceColor.Blue) SimpleShooterMath.turretCompOffset -= 3/255f;
                                            else SimpleShooterMath.turretCompOffset += 3/255f;
                                            SimpleShooterMath.velOffset += 55;
                                        }),
                                        robot.autoShoot(),
                                        intake()
                                ),
                                new WaitUntil(() -> robot.drivetrain.lastPathInChain())
                        ),
                        new Parallel(
                                new Instant(() -> {robot.intake.stopIntake(); SimpleShooterMath.turretCompOffset = 0;}),
                                new Sequential(
                                        new WaitUntil(() -> robot.drivetrain.lastPathInChain() && robot.drivetrain.follower.getTotalDistanceRemaining() <
                                                robot.drivetrain.follower.getCurrentPathChain().length() * 0.5),
                                        robot.gate.open()
                                )
                        )
                )
        );
    }

    public ICommand turretCompShot() {
        return new Parallel(
                new Loop(
                        new Sequential(
                                new Wait(30),
                                new Instant(() -> {
                                    if (Globals.allianceColor == AllianceColor.Blue) SimpleShooterMath.turretCompOffset -= 5/255f;
                                    else SimpleShooterMath.turretCompOffset += 5/255f;
                                })
                        ),
                        5
                ),
                robot.autoShoot()
        );
    }

    public ICommand gateIntake(boolean isInitial) {
        return new Parallel(
                robot.drivetrain.follow(),
                new Sequential(
                        new Instant(() -> {
                            SimpleShooterMath.velOffset = 0;
                            if (Globals.allianceColor == AllianceColor.Blue) SimpleShooterMath.turretCompOffset += 2/255f;
                            else SimpleShooterMath.turretCompOffset -= 2/255f;
                        }),
                        turretCompShot(),
                        new Race(
                                intake(),
                                new Sequential(
                                        new WaitUntil(() -> robot.drivetrain.isDoneFollowing()),
                                        new Race(
                                                new Wait(GATE_WAIT),
                                                new Sequential(
                                                        new WaitUntil(() -> robot.intake.hasTwo()),
                                                        new Wait(600)
                                                )
                                        )
                                )
                        )
                )
        );
    }

    public ICommand gateReturn() {
        return new Parallel(
                robot.drivetrain.followNext(d -> d.tValueCondition(0.95), 2000),
                new Instant(() -> SimpleShooterMath.turretCompOffset = 0/255f),
                new Sequential(
                        new WaitUntil(() -> robot.drivetrain.tValueCondition(0.2)),
                        new Instant(robot.intake::stopIntake),
                        new WaitUntil(() -> robot.drivetrain.tValueCondition(0.5)),
                        robot.gate.open()
                )
        );
    }

    public ICommand thirdSpike() {
        return new Parallel(
                new Sequential(
                        new Wait(400),
                        robot.drivetrain.follow()
                ),
                new Sequential(
                        robot.autoShoot(),
                        new Instant(() -> SimpleShooterMath.hoodCompOffset = 0),
                        robot.resetShooter(),
                        new Instant(() -> {
                            robot.intake();
                            robot.feederWheel.intakeSlow();
                        }),
                        new WaitUntil(() -> robot.drivetrain.tValueCondition(0.5)),
                        new Instant(() -> {robot.intake.distanceSensors.clear(); SimpleShooterMath.turretCompOffset = 0; SimpleShooterMath.velOffset = 0;}),
                        new Sequential(
                                new WaitUntil(() -> robot.drivetrain.lastPathInChain() && robot.drivetrain.tValueCondition(0.5)),
                                new Instant(() -> {robot.intake.stopIntake(); robot.feederWheel.stop();})
                        )

                )
        );
    }

    public ICommand finalCornerCycle() {
        return new Sequential(
                new Parallel(
                        new Sequential(
                                new Wait(400),
                                robot.drivetrain.follow()
                        ),
                        new Sequential(
                                robot.autoShoot(),
                                new Instant(() -> SimpleShooterMath.hoodCompOffset = 0),
                                robot.resetShooter(),
                                new Instant(() -> {
                                    robot.intake();
                                    robot.feederWheel.intakeSlow();
                                    SimpleShooterMath.turretCompOffset = 0;
                                    TrackingThread.trackTurret = false;
                                    robot.turret.setPosition(TURRET_PRESET.getPos());
                                }),
                                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.5)),
                                new Instant(() -> {
                                    robot.intake.distanceSensors.clear();
                                    TrackingThread.trackHood = false;
                                    robot.flywheel.far();
                                    robot.hood.far();
                                }),
                                new WaitUntil(() -> robot.drivetrain.lastPathInChain() && robot.drivetrain.tValueCondition(0.75))
                        )
                ),
                new Instant(() -> TrackingThread.trackTurret = true),
                new Parallel(
                        new Wait(300),
                        robot.gate.open()
                ),
                new Instant(() -> robot.feederWheel.start()),
                new Wait(500)
        );
    }

    public ICommand partnerIntake() {
        return new Sequential(
                robot.drivetrain.follow(),
                new Wait(800)
        );
    }
}
