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
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurretMTI;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.TrackingThread;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.opmodes.paths.CloseAutoPathsMTI;
import org.firstinspires.ftc.teamcode.utils.math.projectile.SimpleShooterMath;

public class CloseAuto extends OpModeCommand {
    private Robot robot;
    private final ElapsedTime matchTimer = new ElapsedTime();
    private TrackingThread autoTrack;
    private boolean useTrack = true;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, new CloseAutoPathsMTI());
        autoTrack = new TrackingThread(robot);
        robot.turret.setPosition(ServoTurretMTI.CLOSE_AUTO_PRELOADS == 0 ? ServoTurretMTI.REST : ServoTurretMTI.CLOSE_AUTO_PRELOADS);
        robot.hood.unsortedAuto();
        IntakeDistanceSensors.useSensors = true;
        TrackingThread.velocityCompensation = false;

        schedule(
                new Infinite(() -> {
                    robot.update();
                    Drivetrain.startPose = robot.drivetrain.follower.getPose();
                    if (useTrack) autoTrack.update();
                }),
                new Sequential(
                        new WaitUntil(() -> !opModeInInit()),
                        new Instant(() -> {
                            robot.flywheel.setVelocity(Flywheel.UNSORTED_AUTO_VELOCITY - 35);
                            matchTimer.reset();
                        }),
                        shootFirstThree(),
                        new Parallel(
                                intake(1),
                                new Race(
                                        new Sequential(
                                                robot.drivetrain.follow(),
                                                robot.drivetrain.follow()
                                        ),
                                        new WaitUntil(() -> robot.intake.hasThree())
                                )
                        ),
                        new Parallel(
                                robot.drivetrain.follow(),
                                transfer(),
                                new Instant(() -> {
                                    robot.flywheel.setVelocity(Flywheel.MEDIUM_VELOCITY - 65);
                                    robot.hood.near();
                                })
                        ),
                        robot.autoShoot(),
                        gateCycle(2),
                        gateCycle(3),
                        gateCycle(4),
                        gateCycle(5),
                        new Parallel(
                                intake(6),
                                new Race(
                                        robot.drivetrain.follow(),
                                        new WaitUntil(() -> robot.intake.hasThree())
                                )
                        ),
                        new Parallel(
                                robot.drivetrain.follow(),
                                transfer(),
                                new Instant(() -> {
                                    robot.hood.setPosition(Hood.CLOSE_AUTO_FINAL);
                                    robot.flywheel.setVelocity(Flywheel.NEAR_VELOCITY - 30);
                                }),
                                new Sequential(
                                        new WaitUntil(() -> robot.drivetrain.tValueCondition(0.9)),
                                        robot.autoShoot()
                                )
                        ),
                        new Instant(() -> robot.intake.intakeMotor.stop())
                )
        );
    }

    public ICommand gateCycle(int i) {
        return new Sequential(
                new Parallel(
                        intakeFromGate(i),
                        robot.drivetrain.followNext(d -> d.velocityCondition(10), 2000)
                ),
                new Parallel(
                        robot.drivetrain.follow(),
                        shootFromGate(i)
                )
        );
    }

    public ICommand shootFirstThree() {
        return new Deadline(
                new Sequential(
                        new WaitUntil(() -> robot.drivetrain.tValueCondition(0.4)),
                        new Parallel(
                                robot.autoShoot(),
                                new Sequential(
                                        new Wait(150),
                                        new Instant(() -> {robot.turret.setPosition(
                                                robot.turret.getPosition() +
                                                        1/255f * (int) Math.signum(ServoTurretMTI.REST - robot.turret.getPosition())
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
                new Instant(() -> robot.intake()),
                new Wait(300),
                new Instant(() -> aimTurret(i))
        );
    }

    public ICommand transfer() {
        return new Sequential(
                new Race(
                        new Wait(600),
                        new WaitUntil(() -> robot.intake.hasThree())
                ),
                robot.transfer()
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
                new Wait(300),
                new Parallel(
                        resetShooter(),
                        new Instant(() -> aimTurret(i)),
                        new Sequential(
                                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.75)),
                                new Instant(() -> robot.intake())
                        )
                ),
                new Race(
                        new Wait(2500),
                        new Sequential(
                                new Wait(1400),
                                new WaitUntil(() -> robot.intake.numBalls() >= 1)
                        ),
                        new WaitUntil(() -> robot.intake.hasThree())
                )
        );
    }

    public ICommand shootFromGate(int i) {
        return new Sequential(
                new Instant(() -> robot.flywheel.setVelocity(Flywheel.MEDIUM_VELOCITY - 65)),
                robot.transfer(),
                new WaitUntil(() -> robot.drivetrain.tValueCondition(0.9)),
                robot.autoShoot()
        );
    }

    public void aimTurret(int i) {
        switch (i) {
            case 1 -> {
                robot.turret.setPosition(ServoTurretMTI.CLOSE_AUTO_SET_1);
                useTrack = false;
            }
            case 6 -> {
                TrackingThread.TURRET_OFFSET = -1/255f;
                useTrack = true;
            }
            default -> {
                TrackingThread.TURRET_OFFSET = -2/255f;
                useTrack = true;
            }
        }
    }
}