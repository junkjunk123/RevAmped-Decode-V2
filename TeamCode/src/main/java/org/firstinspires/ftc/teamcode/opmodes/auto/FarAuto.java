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
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeDistanceSensors;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.TrackingThread;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.opmodes.paths.FarAutoPathsMTI;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.commands.Lazy;
import org.firstinspires.ftc.teamcode.utils.math.projectile.SimpleShooterMath;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicBoolean;

public class FarAuto extends OpModeCommand {
    //idt skip works
    private Robot robot;
    private final ElapsedTime matchTimer = new ElapsedTime();
    private TrackingThread autoTrack;
    public static int SHOOT_DELAY;
    public static int FLYWHEEL_RAMP_UP_WAIT;
    private boolean useTrack = true;
    private final AtomicBoolean skipSweep = new AtomicBoolean(false);

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, new FarAutoPathsMTI());
        autoTrack = new TrackingThread(robot);
        robot.turret.resetTurret();
        robot.hood.far();
        IntakeDistanceSensors.useSensors = true;
        TrackingThread.velocityCompensation = false;
        TrackingThread.trackHood = false; // for ramp-up
        Drivetrain.startPose = robot.drivetrain.follower.getPose();
        resetOffset();
        robot.gate.setGateOpen();

        schedule(
                new Infinite(() -> {
                    robot.update();
                    if (useTrack) autoTrack.update();
                    telemetry.addData("skip",skipSweep.get());
                    telemetry.addData("states", Arrays.toString(robot.intake.getStates()));
                    telemetry.addData("size", robot.drivetrain.getPaths().size());
                    telemetry.addData("turret offset",SimpleShooterMath.turretCompOffset);
                    telemetry.addData("isTeleop",Globals.isTeleOp);
                    telemetry.addData("alliance",Globals.allianceColor);
                    telemetry.update();
                }),
                new Sequential(
                        new WaitUntil(() -> !opModeInInit()),
                        new Instant(() -> {
                            matchTimer.reset();
                            robot.flywheel.far();
                            SimpleShooterMath.turretFarOffset -= 2/255f * Math.signum(SimpleShooterMath.turretFarOffset);
                        }),
                        shootPreloads(),

                        new Instant(this::resetOffset),
                        cycleSpike(),
                        cycleSpike(),

                        cycleSweep(),
                        cycleSweep(),
                        cycleSweep(),
                        cycleSweep(),
                        cycleSweep(),

                        park()

                )
        );

    }

    public ICommand shootPreloads() {
        return new Sequential(
            new Wait(FLYWHEEL_RAMP_UP_WAIT),
            new Instant(() -> {
                robot.hood.far();
            }),
            shoot()
        );
    }

    public ICommand intakeSpike() {
        return new Sequential(
                robot.resetAfterShooting(),
                new Instant(robot::intake),
                //clear the states at 50% of path to remove any false positives from the previous shoot
                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.5)),
                new Instant(() -> robot.intake.distanceSensors.clear()),
                new Parallel(
                        new Sequential(
                                new WaitUntil(() -> robot.intake.ballInTransfer()),
                                new Instant(robot::stopFeeder)
                        ),
                        new Sequential(
                                //new WaitUntil(() -> robot.intake.hasTwo()),
                                new WaitUntil(() -> robot.intake.hasThree()),
                                new Instant(robot::stopIntake)
                        )
                )
        );
    }

    public ICommand shoot() {
        return robot.autoShootFar();
    }

    public ICommand cycleSpike(){
        return new Sequential(
            new Race(
                robot.drivetrain.follow(),
                intakeSpike()
            ),
            new Parallel(
                robot.drivetrain.follow(),
                new Parallel(
                    new Sequential(
                        new WaitUntil(() -> robot.drivetrain.tValueCondition(0.5)),
                        new Instant(robot::stopIntake),
                        robot.gate.open()
                    ),
                    new Sequential(
                        new WaitUntil(() -> robot.drivetrain.isDoneFollowing()),
                        new Wait(SHOOT_DELAY),
                        shoot()
                    )
                )
            )
        );
    }

    public ICommand intakeSweepHP() {
        return new Sequential(
                robot.resetAfterShooting(),
                new Instant(robot::intake),
                //clear the states at 50% of path to remove any false positives from the previous shoot
                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.5)),
                new Instant(() -> robot.intake.distanceSensors.clear()),
                new Parallel(
                        new Sequential(
                                new WaitUntil(() -> robot.intake.ballInTransfer()),
                                new Instant(robot::stopFeeder)
                        ),
                        new Sequential(
                                new WaitUntil(() -> robot.intake.hasTwo()),
                                new Instant(() -> skipSweep.set(true))
                        )
                )
        );
    }

    public ICommand prematureShoot(){
        return new Parallel(
            new Sequential(
                new Instant(robot::stopIntake),
                robot.gate.open()
            ),
            new Sequential(
                new WaitUntil(robot.drivetrain::isDoneFollowing),
                new Wait(SHOOT_DELAY),
                shoot()
            )
        );
    }

    public ICommand fullSweep(){
        return new Sequential(
            new Instant(robot::intake),
            new Deadline(
                new WaitUntil(() -> robot.drivetrain.isDoneFollowing()),
                new Sequential(
                        new WaitUntil(() -> robot.intake.ballInTransfer()),
                        new Instant(robot::stopFeeder)
                ),
                new Sequential(
                        new WaitUntil(() -> robot.intake.hasThree()),
                        new Instant(() -> {robot.stopIntake();})
                )
            ),
            new Instant(robot::stopIntake),
            robot.gate.open(),
            new Wait(SHOOT_DELAY),
            shoot()
        );
    }

    public ICommand cycleSweep(){
        return new Sequential(
            new Instant(() -> {
                skipSweep.set(false);
                resetOffset();
            }),
            new Race(
                new Sequential(
                    robot.drivetrain.follow(), //hp intake path
                    new Wait(200)
                ),
                intakeSweepHP()
            ),
            new Lazy(() -> {
                if (skipSweep.get()) {
                    return new Sequential(
                        new Parallel(
                            robot.drivetrain.follow(),
                            prematureShoot()
                        ),
                        new Instant(robot.drivetrain::skip)
                    );
                }

                return new Sequential(
                    new Instant(robot.drivetrain::skip),
                    new Parallel(
                        robot.drivetrain.follow(),
                        fullSweep(),
                        new Instant(() -> SimpleShooterMath.turretFarOffset -= 1/255f * Math.signum(SimpleShooterMath.turretFarOffset))
                    )
                );
            })
        );
    }

    public ICommand park(){
        return robot.drivetrain.followLast(Drivetrain.isDone);
    }

    private void resetOffset() {
        if (Globals.allianceColor.equals(AllianceColor.Red)){
            SimpleShooterMath.turretFarOffset = 3/255f;
        } else {
            SimpleShooterMath.turretFarOffset = -2/255f;
        }
    }
}