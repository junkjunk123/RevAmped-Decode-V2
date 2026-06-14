package org.firstinspires.ftc.teamcode.opmodes.auto;
import android.graphics.drawable.Icon;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Conditional;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Deadline;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeDistanceSensors;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.TrackingThread;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.opmodes.paths.CloseAutoPathsMTI;
import org.firstinspires.ftc.teamcode.opmodes.paths.FarAutoPathsMTI;
import org.firstinspires.ftc.teamcode.utils.math.projectile.SimpleShooterMath;

import java.util.Arrays;

public class FarAuto extends OpModeCommand {
    //idt skip works
    private Robot robot;
    private final ElapsedTime matchTimer = new ElapsedTime();
    private TrackingThread autoTrack;

    public static int FLYWHEEL_RAMP_UP_WAIT;
    private boolean useTrack = true;
    private boolean skipSweep = false;

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
        robot.gate.setGateOpen();

        schedule(
                new Infinite(() -> {
                    robot.update();
                    if (useTrack) autoTrack.update();
                    telemetry.addData("skip",skipSweep);
                    telemetry.addData("states", Arrays.toString(robot.intake.getStates()));
                    telemetry.update();
                }),
                new Sequential(
                        new WaitUntil(() -> !opModeInInit()),
                        new Instant(() -> {
                            matchTimer.reset();
                            robot.flywheel.setPower(1.0f); // ramp up
                        }),
                        shootPreloads(),

                        cycleSpike(),
                        cycleSpike(),

                        cycleSweep(),

                        park()

                )
        );

    }

    public ICommand shootPreloads() {
        return new Sequential(
            new Wait(FLYWHEEL_RAMP_UP_WAIT),
            new Instant(() -> TrackingThread.trackHood = true),
            new Wait(200),
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
        return new Sequential(
                new Instant(robot::transferShootFar),
                new Wait(Robot.SHOOT_TIME_FAR)
            );
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
                        robot.gate.open()
                    ),
                    new Sequential(
                        new WaitUntil(() -> robot.drivetrain.tValueCondition(0.9)),
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
                                new Instant(() -> {
                                    skipSweep = true;
                                    robot.stopIntake();
                                })
                        )
                )
        );
    }

    public ICommand intakeSweepFull(){
        return new Sequential(
            new Instant(robot::intake),
            new Deadline(
                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.9)),
                new Parallel(
                    new Sequential(
                            new WaitUntil(() -> robot.intake.ballInTransfer()),
                            new Instant(robot::stopFeeder)
                    ),
                    new Sequential(
                            new WaitUntil(() -> robot.intake.hasThree()),
                            new Instant(() -> {robot.stopIntake();})
                    )
                )

            )
        );
    }

    public ICommand cycleSweep(){
        return new Sequential(
            new Instant(() -> skipSweep = false),
            new Race(
                new Sequential(
                    robot.drivetrain.follow(),
                    new Wait(250)
                ),
                intakeSweepHP()
            ),
            new Conditional(
                () -> skipSweep,
                new Parallel(
                    robot.drivetrain.follow(),
                    new Sequential(
                        new WaitUntil(() -> robot.drivetrain.tValueCondition(0.9)),
                        shoot(),
                        new Instant(() -> robot.drivetrain.skip())
                    )
                ),
                new Sequential(
                    new Instant(() -> {
                        robot.drivetrain.skip();
                    }),
                    new Parallel(
                        robot.drivetrain.follow(),
                        new Sequential(
                            intakeSweepFull(),
                            shoot()
                        )
                    )
                )
            )
        );
    }

    public ICommand park(){
        return robot.drivetrain.follow();
    }
}