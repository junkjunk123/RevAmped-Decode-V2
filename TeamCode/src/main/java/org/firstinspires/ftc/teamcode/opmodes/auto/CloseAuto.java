package org.firstinspires.ftc.teamcode.opmodes.auto;
import com.pedropathing.ivy.Command;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.opengl.TextResourceReader;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeDistanceSensors;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurretMTI;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.TrackingThread;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.opmodes.paths.CloseAutoPathsMTI;
import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.math.projectile.SimpleShooterMath;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;

public class CloseAuto extends OpModeCommand {
    private Robot robot;
    private final ElapsedTime matchTimer = new ElapsedTime();
    private TrackingThread autoTrack;

    public static int GATE_WAIT;

    public static int flywheel_ramp_vel;
    private boolean useTrack = true;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, new CloseAutoPathsMTI());
        autoTrack = new TrackingThread(robot);
        robot.turret.closeAutoPreloads();
        robot.hood.near();
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
                }),
                new Sequential(
                        new WaitUntil(() -> !opModeInInit()),
                        new Instant(() -> {
                            matchTimer.reset();
                            robot.flywheel.setVelocity(Flywheel.CLOSE_PRELOADS_VEL);
                        }),

                        //Shooting preloads
                        new Parallel(
                                robot.drivetrain.follow(),
                                shootPreloads()
                        ),

                        //First Spike Mark
                        new Parallel(
                                new Instant(
                                        () -> { //enable tracking
                                            TrackingThread.trackTurret = true;
                                            TrackingThread.trackHood = true;
                                        }
                                ),
                                cycle()
                        ),

                        //Gate Cycle 1 & 2
                        gateCycle(),
                        gateCycle(),

                        //Second Spike Mark
                        cycle(),

                        //Gate Cycle 3 & 4 & 5
                        gateCycle(),
                        gateCycle(),
                        gateCycle(),

                        //Park
                        robot.drivetrain.follow()
                )
        );

    }

    public ICommand shootPreloads() {
        return new Sequential(
                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.9)),
                new Wait(150), //to remove/mitigate the slight backwards vel while shooting
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
                                //new WaitUntil(() -> robot.intake.hasTwo()),
                                new WaitUntil(() -> robot.intake.hasThree()),
                                new Instant(robot::stopIntake)
                        )
                )
        );
    }

    public ICommand shoot() {
        return robot.autoShoot();
    }

    public ICommand cycle() {
        return new Sequential(
            //Intaking
            new Race(
                robot.drivetrain.follow(),
                intake()
            ),

            //Shooting
            new Parallel(
                robot.drivetrain.follow(), //shooting path
                new Sequential(
                    new WaitUntil(() -> robot.drivetrain.tValueCondition(0.5)),
                    robot.gate.open()
                )
            ),
            shoot()
        );
    }

    public ICommand gateCycle(){
        return new Sequential(
            //Intaking
            new Parallel(
                robot.drivetrain.follow(),
                new Race(
                    intake(),
                    new Sequential(
                        new WaitUntil(() -> robot.drivetrain.tValueCondition(0.9)),
                        new Wait(GATE_WAIT)
                    )
                )
            ),

            //Shooting
            new Parallel(
                robot.drivetrain.follow(),
                new Sequential(
                    new WaitUntil(() -> robot.drivetrain.tValueCondition(0.5)),
                    robot.gate.open()
                )
            ),
            shoot()
        );
    }
}