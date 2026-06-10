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
        TrackingThread.trackTurret = false; //turning off for preloads preset
        TrackingThread.trackHood = false; //turning off for preloads preset
        Drivetrain.startPose = robot.drivetrain.follower.getPose();
        robot.gate.setGateOpen();

        schedule(
                new Infinite(() -> {
                    robot.update();
                    if (useTrack) autoTrack.update();
                    telemetry.addData("flywheel vel",robot.flywheel.getVelocity());
                    telemetry.update();
                }),
                new Sequential(
                        new WaitUntil(() -> !opModeInInit()),
                        new Instant(() -> {
                            matchTimer.reset();
                            robot.flywheel.setPower(1.0f);
                        }),

                        //Shooting preloads
                        new Parallel(
                            robot.drivetrain.follow(),
                            shootPreloads()
                        ),

                        //First Spike Mark
                        new Parallel(
                            new Instant(() -> {
                                //preloads are done
                                TrackingThread.trackTurret = true;
                                TrackingThread.trackHood = true;
                            }),
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

    public ICommand shootPreloads(){
        return new Sequential(
            new WaitUntil(() -> robot.drivetrain.tValueCondition(0.3)),
            new Instant(robot.flywheel::closePreloadsPreset),
            new WaitUntil(() -> robot.drivetrain.tValueCondition(0.9)),
            shoot()
        );
    }

    public ICommand intake(){
        return new Sequential(
            robot.resetAfterShooting(),
            new Instant(robot::intake),
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
            new Instant(robot::transferShoot),
            new Wait(Robot.SHOOT_TIME)
        );
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
                robot.drivetrain.follow(),
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

//    public ICommand gateCycle(int i) {
//        return new Sequential(
//                new Parallel(
//                        intakeFromGate(i),
//                        robot.drivetrain.followNext(d -> d.velocityCondition(10), 2000)
//                ),
//                new Parallel(
//                        robot.drivetrain.follow(),
//                        shootFromGate(i)
//                )
//        );
//    }
//
//    public ICommand shootFirstThree() {
//        return new Deadline(
//                new Sequential(
//                        new WaitUntil(() -> robot.drivetrain.tValueCondition(0.4)),
//                        new Parallel(
//                                robot.autoShoot(),
//                                new Sequential(
//                                        new Wait(150),
//                                        new Instant(() -> {robot.turret.setPosition(
//                                                robot.turret.getPosition() +
//                                                        1/255f * (int) Math.signum(ServoTurretMTI.REST - robot.turret.getPosition())
//                                        ); robot.flywheel.setVelocity(Flywheel.UNSORTED_AUTO_VELOCITY + 90);})
//                                )
//                        )
//                ),
//                robot.drivetrain.followNext(d -> d.tValueCondition(0.9), 3000)
//        );
//    }
//
//    public ICommand intake(int i) {
//        return new Sequential(
//                resetShooter(),
//                new Instant(() -> robot.intake()),
//                new Wait(300),
//                new Instant(() -> aimTurret(i))
//        );
//    }
//
//    public ICommand transfer() {
//        return new Sequential(
//                new Race(
//                        new Wait(600),
//                        new WaitUntil(() -> robot.intake.hasThree())
//                ),
//                robot.transfer()
//        );
//    }
//
//    public ICommand resetShooter() {
//        return new Instant(() -> {
//            robot.flywheel.stop();
//            robot.feederWheel.stop();
//        });
//    }
//
//    public ICommand intakeFromGate(int i) {
//        return new Sequential(
//                new Wait(300),
//                new Parallel(
//                        resetShooter(),
//                        new Instant(() -> aimTurret(i)),
//                        new Sequential(
//                                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.75)),
//                                new Instant(() -> robot.intake())
//                        )
//                ),
//                new Race(
//                        new Wait(2500),
//                        new Sequential(
//                                new Wait(1400),
//                                new WaitUntil(() -> robot.intake.numBalls() >= 1)
//                        ),
//                        new WaitUntil(() -> robot.intake.hasThree())
//                )
//        );
//    }
//
//    public ICommand shootFromGate(int i) {
//        return new Sequential(
//                new Instant(() -> robot.flywheel.setVelocity(Flywheel.MEDIUM_VELOCITY - 65)),
//                robot.transfer(),
//                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.9)),
//                robot.autoShoot()
//        );
//    }
//
//    public void aimTurret(int i) {
//        switch (i) {
//            case 1 -> {
//                robot.turret.setPosition(ServoTurretMTI.CLOSE_AUTO_SET_1);
//                useTrack = false;
//            }
//            case 6 -> {
//                TrackingThread.TURRET_OFFSET = -1/255f;
//                useTrack = true;
//            }
//            default -> {
//                TrackingThread.TURRET_OFFSET = -2/255f;
//                useTrack = true;
//            }
//        }
//    }
}