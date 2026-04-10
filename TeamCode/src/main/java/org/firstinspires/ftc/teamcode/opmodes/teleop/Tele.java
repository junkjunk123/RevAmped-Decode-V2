package org.firstinspires.ftc.teamcode.opmodes.teleop;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.math.projectile.SimpleShooterMath;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.TeleOpStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeThread;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Popper;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.TrackingThread;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.pedro.ColoredDecodePose;
import org.firstinspires.ftc.teamcode.utils.GamepadEx;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RandomizationState;
import org.firstinspires.ftc.teamcode.utils.Z3Element;
import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.commands.Lazy;
import org.firstinspires.ftc.teamcode.utils.prompter.Prompter;
import org.firstinspires.ftc.teamcode.utils.prompter.StatePrompt;

import java.util.Arrays;

@Config
@TeleOp(name = "DCTeleOp")
public class Tele extends OpModeCommand {
    private GamepadEx gamepad_1;
    private GamepadEx gamepad_2;
    private Robot robot;
    private TeleOpStateHandler tsh;
    private Prompter prompter;
    private boolean transfer;
    private boolean canShoot = true;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        tsh = RobotStateHandler.createTeleOpStateHandler(robot);
        gamepad_1 = new GamepadEx(gamepad1);
        gamepad_2 = new GamepadEx(gamepad2);
        prompter = new Prompter(this, gamepad_1)
                .prompt("motif", new StatePrompt<>("Select the motif pattern", RandomizationState.class))
                .onComplete(() -> Globals.randomizationState = prompter.getOrDefault("motif", Globals.randomizationState))
                .thenDisplay("Good luck! We're rooting for you. --- Havish & Eric");
        
        gamepad_1.left_trigger_button(f -> f.greaterThan(0.3f));
        gamepad_1.right_trigger_button(f -> f.greaterThan(0.3f));
        gamepad_2.left_trigger_button(f -> f.greaterThan(0.3f));
        gamepad_2.right_trigger_button(f -> f.greaterThan(0.3f));

        // Schedule robot update loop
        schedule(new Infinite(() -> {
            robot.update();
            if (!robot.drivetrain.isHoldingPose()) robot.drivetrain.arcadeDrive(gamepad1);
        }));

        // Initialize robot
        schedule(new Sequential(
                new WaitUntil(() -> !opModeInInit()),
                new Instant(robot::initialize),
                new Wait(500),
                tsh.runTransition(() -> {}, RobotStateHandler.CycleState.SHOOT),
                tsh.runTransition(
                    new Sequential(
                        robot.shootAll(),
                        new Parallel(
                                robot.resetShooter(),
                                new Sequential(
                                        new Instant(robot.intakeMotor::stop),
                                        robot.table.reset(),
                                        new Instant(() -> {
                                            robot.intakeMotor.intake();
                                            robot.feederWheel.setIntake();
                                        }),
                                        new Parallel(
                                                robot.popper.neutral(),
                                                robot.intakeGate.open(),
                                                new Sequential(
                                                        new Wait(300),
                                                        robot.splitter.activate()
                                                )
                                        )
                                )
                        )
                    ), RobotStateHandler.CycleState.INTAKE)
            )
        );

        TrackingThread.trackHood = false;
        TrackingThread.trackTurret = false;
    }

    @Override
    public void initializeLoop() {
        telemetry.addData("alliance", Globals.allianceColor);
        telemetry.addData("randomizationState", Globals.randomizationState);
        prompter.run();
    }

    @Override
    public void execute() {
        // Update switches
        gamepad_1.update();
        gamepad_2.update();

        if (RobotStateHandler.CycleState.DRIVE_TO_SHOOT.INSTANCE == RobotStateHandler.DriveState.AUTO_TRACKING) {
            RobotStateHandler.CycleState.DRIVE_TO_SHOOT.update();
        }

        // Schedule commands based on triggers
        if (gamepad_1.a.isRisingEdge()) {
            if (robot.getRobotState().equals(RobotStateHandler.CycleState.INTAKE)) IntakeThread.useSensors = !IntakeThread.useSensors;
            else {
                TrackingThread.trackTurret = !TrackingThread.trackTurret;
                TrackingThread.trackHood = !TrackingThread.trackHood;
            }
        }

        if (gamepad_1.b.isRisingEdge()) {
            schedule(new Sequential(
                        robot.limelight.computeOffsets(),
                        new Instant(() -> Globals.getTrackingThread().addLimelightMeasurement(robot.limelight.getOffsets()))
                    )
            );
        }

        if (gamepad_1.y.isRisingEdge()) {
            if (!robot.drivetrain.isHoldingPose())
                schedule(robot.drivetrain.holdPose());
            else
                robot.drivetrain.stopHoldPose();
        }

        if (gamepad_1.dpad_up.isRisingEdge()) schedule(tsh.setting(robot::shootCorner));
        if (gamepad_1.dpad_down.isRisingEdge()) schedule(tsh.setting(robot::shootNear));
        if (gamepad_1.dpad_left.isRisingEdge()) schedule(tsh.setting(robot::shootMedium));
        if (gamepad_1.dpad_right.isRisingEdge()) schedule(tsh.setting(robot.shootFar()));

        if (gamepad_1.right_bumper.isRisingEdge()) {
            if (TrackingThread.trackTurret) {
                SimpleShooterMath.APRIL_TAG_POSE_BLUE = SimpleShooterMath.APRIL_TAG_POSE_BLUE.plus(new Pose(3, 3));
                SimpleShooterMath.APRIL_TAG_POSE_RED = SimpleShooterMath.APRIL_TAG_POSE_RED.plus(new Pose(3, -3));
            } else {
                schedule(tsh.task(() -> robot.turret.previous(), new int[]{1, 1, 0}));
            }
        }

        if (gamepad_1.left_bumper.isRisingEdge()) {
            if (TrackingThread.trackTurret) {
                SimpleShooterMath.APRIL_TAG_POSE_BLUE = SimpleShooterMath.APRIL_TAG_POSE_BLUE.plus(new Pose(3, 3));
                SimpleShooterMath.APRIL_TAG_POSE_RED = SimpleShooterMath.APRIL_TAG_POSE_RED.plus(new Pose(3, -3));
            } else {
                schedule(tsh.task(() -> robot.turret.next(), new int[]{1, 1, 0}));
            }
        }

        if (gamepad_1.x.isRisingEdge()) schedule(tsh.override(
                new Parallel(
                        robot.intakeGate.open(),
                        robot.popper.neutral(),
                        robot.splitter.activate()
                ), RobotStateHandler.IntakeMessage.SORTING)
        );

        if (gamepad_2.b.isRisingEdge() && (tsh.atState(RobotStateHandler.CycleState.DRIVE_TO_SHOOT) || !robot.intakeMotor.atState(IntakeMotor.IntakeState.INTAKE))) {
            schedule(tsh.runTransition(new Parallel(
                    new Instant(() -> {
                        robot.flywheel.stop();
                        robot.intakeTilt.intake();
                        robot.feederWheel.stop();
                    }),
                    new Sequential(
                            robot.resetTable(),
                            new Parallel(
                                    robot.intakeGate.open(),
                                    robot.splitter.activate()
                            )
                    ),
                    new Lazy(() -> {
                        if (TrackingThread.trackTurret) return robot.turret.resetTurret();
                        else return Commands.NOOP;
                    })
            ), RobotStateHandler.CycleState.INTAKE));
        }

        if (gamepad_2.y.isRisingEdge()) schedule(robot.sort());
        if (gamepad_2.right_trigger_button.isRisingEdge()) {
            if (robot.intakeMotor.atState(IntakeMotor.IntakeState.OUTTAKE)) robot.intakeMotor.stop();
            else robot.intakeMotor.outtake();
            robot.intakeTilt.transfer();
        }
        if (gamepad_2.a.isRisingEdge()) tsh.setState(RobotStateHandler.CycleState.DRIVE_TO_SHOOT);
        if (gamepad_2.right_bumper.isRisingEdge()) schedule(tsh.task(
                new Sequential(
                        new Instant(() -> robot.setRobotState(RobotStateHandler.IntakeMessage.SORTING)),
                        new Parallel(
                                robot.splitter.activate(),
                                robot.intakeGate.close()
                        ),
                        robot.table.previous()
                ), new int[]{1, 0, 0}
        ));

        if (gamepad_2.left_bumper.isRisingEdge()) schedule(tsh.task(
                new Sequential(
                        new Instant(() -> robot.setRobotState(RobotStateHandler.IntakeMessage.SORTING)),
                        new Parallel(
                                robot.intakeGate.close(),
                                robot.splitter.activate()
                        ),
                        robot.table.next()
                ), new int[]{1, 0, 0}
        ));

        if (IntakeThread.useSensors && tsh.atState(RobotStateHandler.CycleState.INTAKE) && robot.intakeMotor.getPower() > 0.2) {
            if (robot.tableCompartments.intakeThread.getNumBalls() == 3) {
                robot.tableCompartments.populate();
                transfer = true;
                RobotStateHandler.CycleState.INTAKE.update = false;
            }
        }

        if (transfer || (gamepad_2.x.isRisingEdge() && tsh.atState(RobotStateHandler.CycleState.INTAKE) && robot.popper.atState(Popper.PopperState.NEUTRAL))) {
            schedule(tsh.runTransition(
                    new Parallel(
                            new Instant(() -> {
                                transfer = false;
                                robot.intakeTilt.transfer();
                                RobotStateHandler.CycleState.INTAKE.update = false;
                            }),
                            new Sequential(
                                    new Wait(200),
                                    new Instant(robot.intakeMotor::outtake)
                            ),
                            robot.popper.pop(),
                            robot.splitter.neutral(),
                            robot.intakeGate.close(),
                            new Conditional(
                                    robot.flywheel::isStopped,
                                    new Instant(robot::shootNear),
                                    Commands.NOOP
                            )
                    ),
                    RobotStateHandler.CycleState.DRIVE_TO_SHOOT
            ));
        }

        if (gamepad_2.dpad_up.isRisingEdge()) {
            schedule(new Conditional(
                    () -> tsh.evaluate(RobotStateHandler.CycleState.SHOOT) && canShoot,
                    new Sequential(
                            new Instant(() -> canShoot = false),
                            tsh.runTransition(() -> {}, RobotStateHandler.CycleState.SHOOT),
                            tsh.runTransition(new Sequential(
                                    new Race(
                                            new WaitUntil(gamepad_2.x::isRisingEdge),
                                            new Sequential(
                                                    robot.shootAll(),
                                                    new Instant(() -> canShoot = true)
                                            )
                                    ),
                                    robot.resetAfterShooting()
                            ), RobotStateHandler.CycleState.INTAKE)
                    ),
                    Commands.NOOP
            ));
        }

        if (gamepad_2.dpad_down.isRisingEdge()) {
            schedule(new Conditional(
                            () -> tsh.evaluate(RobotStateHandler.CycleState.SHOOT),
                            new Sequential(
                                    tsh.runTransition(() -> {}, RobotStateHandler.CycleState.SHOOT),
                                    tsh.runTransition(new Sequential(
                                            robot.shootAll(Table.SLOW_SHOOT_DELAY),
                                            robot.resetAfterShooting()
                                    ), RobotStateHandler.CycleState.INTAKE)
                            ),
                            Commands.NOOP
                    )
            );
        }

        if (gamepad_2.dpad_left.isRisingEdge()) {
            schedule(tsh.task(robot.popper.block(), RobotStateHandler.CycleState.INTAKE));
        } else if (gamepad_2.dpad_left.isFallingEdge()) {
            schedule(tsh.task(robot.popper.neutral(), RobotStateHandler.CycleState.INTAKE));
        }

        if (gamepad_2.dpad_right.isRisingEdge()) {
            robot.drivetrain.follower.setPose(new ColoredDecodePose(72, 130.5, Math.PI).getPose());
        }

        if (gamepad2.left_stick_y < -0.3f) {
            robot.intakeTilt.transfer();
        } else if (gamepad2.left_stick_y > 0.3f) {
            robot.intakeTilt.intake();
        }

        telemetry.addData("tableEncoderPos", robot.table.getEncoder().getPosition());
        telemetry.addData("tableEncoderVel", robot.table.getEncoder().getVelocity());
        telemetry.addData("Pose",robot.drivetrain.follower.getPose());
        telemetry.update();
    }
}