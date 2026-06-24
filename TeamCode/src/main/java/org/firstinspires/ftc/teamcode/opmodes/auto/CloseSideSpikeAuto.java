package org.firstinspires.ftc.teamcode.opmodes.auto;
import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Lazy;
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
import org.firstinspires.ftc.teamcode.mechanisms.shooter.TrackingThread;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.opmodes.paths.CloseSideSpikeAutoPathsMTI;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.math.projectile.SimpleShooterMath;

import java.util.concurrent.atomic.AtomicBoolean;

public class CloseSideSpikeAuto extends OpModeCommand {
    private Robot robot;
    private final ElapsedTime matchTimer = new ElapsedTime();
    private TrackingThread autoTrack;

    public static int GATE_WAIT;

    public static int flywheel_ramp_vel;
    private boolean useTrack = true;
    private AtomicBoolean stop = new AtomicBoolean();

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, new CloseSideSpikeAutoPathsMTI());
        autoTrack = new TrackingThread(robot);
        robot.turret.closeSideSpikePreloads();
        robot.hood.setPosition(0.1);
        IntakeDistanceSensors.useSensors = true;
        TrackingThread.velocityCompensation = false;
        TrackingThread.trackHood = false; //for preloads preset
        TrackingThread.trackTurret = false; // for preloads preset
        Drivetrain.startPose = robot.drivetrain.follower.getPose();
        robot.gate.setGateOpen();

        schedule(
                new Infinite(() -> {
                    robot.update();
                    if (useTrack) autoTrack.update();
                    telemetry.update();
                }),
                new Sequential(
                        new WaitUntil(() -> !opModeInInit()),
                        new Instant(() -> {
                            matchTimer.reset();
                            robot.flywheel.setVelocity(Flywheel.CLOSE_PRELOADS_VEL);
                        }),

                        //Shooting preloads
                        /*
                        new Parallel(
                                robot.drivetrain.follow(),
                                shootPreloads()
                        ),
                         */

                        robot.drivetrain.follow(),
                        shoot(),

                        //First Spike Mark
                        new Parallel(
                                new Instant(
                                        () -> { //enable tracking
                                            TrackingThread.trackTurret = true;
                                            TrackingThread.trackHood = true;
                                        }
                                ),
                                sideSpikeCycle()
                        ),

                        //Second Spike Mark
                        spikeCycle(),

                        //Gate Cycle 1 & 2
                        gateCycle(),
                        gateCycle(),

                        //Gate Cycle 3 & 4
                        gateCycle(),
                        gateCycle(),
                        gateCycle(),
                        gateCycle(),
                        shootLast()
                )
        );

    }

    public ICommand shootPreloads() {
        return new Sequential(
                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.9)),
                new Wait(250), //ramp up
                shoot()
        );
    }

    public ICommand intake() {
        return new Sequential(
                new Instant(() -> SimpleShooterMath.hoodCompOffset = 0),
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
                                new WaitUntil(() -> robot.drivetrain.isDoneFollowing() || robot.drivetrain.follower.atParametricEnd()),
                                new WaitUntil(() -> robot.intake.hasTwo())
                        )
                )
        );
    }

    public ICommand shoot() {
        return robot.autoShoot();
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
            ),
            shoot()
        );
    }

    public ICommand spikeCycle() {
        return new Sequential(
                //Intaking
                new Race(
                        robot.drivetrain.follow(),
                        intake()
                ),
                //Shooting
                new Parallel(
                        robot.drivetrain.follow(), //shooting path
                        new Instant(() -> robot.intake.stopIntake()),
                        new Sequential(
                                new WaitUntil(() -> robot.drivetrain.follower.getChainIndex() ==
                                        robot.drivetrain.follower.getCurrentPathChain().size() - 1 &&
                                        robot.drivetrain.follower.getTotalDistanceRemaining() <
                                                robot.drivetrain.follower.getCurrentPathChain().length() * 0.5),
                                robot.gate.open()
                        )
                )
        );
    }

    public ICommand gateCycle() {
        return new Conditional(
                () -> stop.get(),
                Commands.NOOP,
                new Sequential(
                    gateIntakeAndShoot(),
                    toShootPoint()
                )
        );
    }

    public ICommand shootLast(){
        return new Sequential(
            new WaitUntil(() -> !stop.get()),
            new Instant(() -> SimpleShooterMath.turretCompOffset = 0),
            shoot()
        );
    }

    public ICommand toShootPoint(){
        return new Parallel(
            robot.drivetrain.followNext(d -> d.tValueCondition(0.95), 2000),
            new Instant(() -> SimpleShooterMath.turretCompOffset = 0/255f),
            new Sequential(

                new WaitUntil(() -> robot.drivetrain.follower.getTotalDistanceRemaining() <
                        robot.drivetrain.follower.getCurrentPathChain().length() * 0.8),
                new Instant(robot.intake::stopIntake),

                new WaitUntil(() -> robot.drivetrain.follower.getTotalDistanceRemaining() <
                                robot.drivetrain.follower.getCurrentPathChain().length() * 0.5),
                robot.gate.open()
            )
        );
    }

    public ICommand gateIntakeAndShoot(){
        return new Parallel(
            robot.drivetrain.follow(),
            new Sequential(
                new Instant(() -> {
                    if (Globals.allianceColor == AllianceColor.Blue) SimpleShooterMath.turretCompOffset += 2/255f;
                    else SimpleShooterMath.turretCompOffset -= 2/255f;
                }),
                new Parallel(
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
                    shoot()
                ),
                new Race(
                        intake(),
                        new Sequential(
                            new WaitUntil(() -> robot.drivetrain.isDoneFollowing()),
                            new Wait(GATE_WAIT)
                        ),
                        new Sequential(
                                new WaitUntil(() -> matchTimer.seconds() > 29.5),
                                new Instant(() -> stop.set(true)),
                                robot.drivetrain.followLast(Drivetrain.isDone)
                        )
                )
            )
        );
    }

}