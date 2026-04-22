package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.TeleOpStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeThread;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Popper;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.GyroThread;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.pedro.ColoredDecodePose;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.commands.GamepadEx;
import org.firstinspires.ftc.teamcode.utils.commands.RandomizationState;
import org.firstinspires.ftc.teamcode.utils.data.BooleanSwitch;
import org.firstinspires.ftc.teamcode.utils.data.FloatSupplier;
import org.firstinspires.ftc.teamcode.utils.math.projectile.TrackState;
import org.firstinspires.ftc.teamcode.utils.prompter.Prompter;
import org.firstinspires.ftc.teamcode.utils.prompter.StatePrompt;

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
    private GyroThread gyroThread;
    private boolean firstTime = true;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        tsh = RobotStateHandler.createTeleOpStateHandler(robot);
        gamepad_1 = new GamepadEx(gamepad1);
        gamepad_2 = new GamepadEx(gamepad2);
        gyroThread = new GyroThread(robot);
        gyroThread.setState(TrackState.REST);
        prompter = new Prompter(this, gamepad_1)
                .prompt("motif", new StatePrompt<>("Select the motif pattern", RandomizationState.class))
                .prompt("alliance", new StatePrompt<>("Select the alliance color", AllianceColor.class))
                .onComplete(() -> {
                    Globals.randomizationState = prompter.getOrDefault("motif", Globals.randomizationState);
                    Globals.allianceColor = prompter.getOrDefault("alliance", Globals.allianceColor);
                })
                .thenDisplay("Good luck! We're rooting for you. --- Havish & Eric");
        
        gamepad_1.left_trigger_button(FloatSupplier::isPress);
        gamepad_1.right_trigger_button(FloatSupplier::isPress);
        gamepad_2.left_trigger_button(FloatSupplier::isPress);
        gamepad_2.right_trigger_button(FloatSupplier::isPress);
        gamepad_2.left_stick_y_button(FloatSupplier::isPress);
        gamepad_2.right_stick_y_button(FloatSupplier::isPress);

        // Schedule robot update loop
        schedule(new Infinite(() -> {
            robot.update();
            if (!robot.drivetrain.isHoldingPose()) robot.drivetrain.arcadeDrive(gamepad1);
            gyroThread.update();
            telemetry.update();
        }));

        tsh.setState(RobotStateHandler.CycleState.DRIVE_TO_SHOOT);

        // Initialize robot
        schedule(new Sequential(
                new WaitUntil(() -> !opModeInInit()),
                new Instant(robot::initialize),
                new Wait(500),
                tsh.runTransition(() -> {}, RobotStateHandler.CycleState.SHOOT),
                tsh.runTransition(
                        new Sequential(
                                robot.popper.neutral(),
                                robot.shootAll(),
                                new Parallel(
                                        robot.resetShooter(),
                                        new Sequential(
                                                new Parallel(
                                                        new Instant(robot.intakeMotor::stop),
                                                        robot.intakeGate.open(),
                                                        robot.table.reset()),
                                                new Instant(() -> {
                                                    robot.intakeMotor.intake();
                                                    robot.feederWheel.intakeState();
                                                }),
                                                new Parallel(
                                                        robot.popper.neutral(),
                                                        new Sequential(
                                                                new Wait(300),
                                                                robot.splitter.activate()
                                                        )
                                                )
                                        )
                                )
                        ),
                RobotStateHandler.CycleState.INTAKE)
            )
        );
    }

    @Override
    public void initializeLoop() {
        telemetry.addData("alliance", Globals.allianceColor);
        telemetry.addData("randomizationState", Globals.randomizationState);
        prompter.run();
    }

    @Override
    public void onStart() {
        robot.drivetrain.follower.setPose(Drivetrain.startPose);
        robot.drivetrain.follower.update();
    }

    @Override
    public void execute() {
        // Update switches
        gamepad_1.update();
        gamepad_2.update();

        // Schedule commands based on triggers
        if (gamepad_1.a.isRisingEdge()) {
            GyroThread.trackTurret = !GyroThread.trackTurret;
            GyroThread.trackTraj = !GyroThread.trackTraj;
        }

        if (gamepad_1.y.isRisingEdge()) {
            if (!robot.drivetrain.isHoldingPose()) schedule(robot.drivetrain.holdPose());
            else robot.drivetrain.stopHoldPose();
        }

        if (gamepad_1.dpad_up.isRisingEdge()){
            if (!gyroThread.isFar()) gyroThread.setState(TrackState.CLOSE_THREE);
            else gyroThread.setState(TrackState.FAR_THREE);
            schedule(tsh.setting(robot::shootCorner),
                    new Instant(gyroThread::close));
        }
        if (gamepad_1.dpad_down.isRisingEdge()) schedule(tsh.setting(robot::shootNear), new Instant(gyroThread::close));
        if (gamepad_1.dpad_left.isRisingEdge()) schedule(tsh.setting(robot::shootMedium), new Instant(gyroThread::close));
        if (gamepad_1.dpad_right.isRisingEdge()) schedule(tsh.setting(robot::shootFar), new Instant(gyroThread::far));

        if (gamepad_1.start.isRisingEdge()){
            schedule(robot.turret.resetTurret());
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
                            new Conditional(
                                    () -> !firstTime,
                                    robot.resetTableTeleOp(),
                                    new Sequential(
                                            new Instant(() -> firstTime = false),
                                            new Parallel(
                                                    new Instant(robot.intakeMotor::stop),
                                                    robot.intakeGate.open(),
                                                    robot.table.reset()),
                                            new Instant(() -> {
                                                robot.intakeMotor.intake();
                                                robot.feederWheel.intakeState();
                                            }),
                                            new Parallel(
                                                    new Sequential(
                                                            robot.popper.neutral(),
                                                            new Wait(200)
                                                    ),
                                                    robot.splitter.activate()
                                            )
                                    )
                            ),
                            new Parallel(
                                    robot.intakeGate.open(),
                                    robot.splitter.activate()
                            )
                    )
            ), RobotStateHandler.CycleState.INTAKE));
        }

        if (gamepad_2.y.isRisingEdge()) schedule(robot.sort());
        if (gamepad_2.right_trigger_button.isRisingEdge()) {
            if (robot.intakeMotor.atState(IntakeMotor.IntakeState.OUTTAKE)) robot.intakeMotor.stop();
            else robot.intakeMotor.outtake();
            robot.intakeTilt.transfer();
        }

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
            if (robot.tableCompartments.intakeThread.hasThree) {
                robot.tableCompartments.populate();
                transfer = true;
                RobotStateHandler.CycleState.INTAKE.update = false;
                robot.intakeTilt.transfer();
                robot.intakeMotor.outtake();
            }
        }

        if (transfer || (gamepad_2.x.isRisingEdge() && tsh.atState(RobotStateHandler.CycleState.INTAKE) && robot.popper.atState(Popper.PopperState.NEUTRAL))) {
            transfer = false;
            schedule(new Sequential(
                        tsh.runTransition(
                                new Parallel(
                                        new Instant(() -> robot.feederWheel.start()),
                                        robot.popper.pop(),
                                        robot.splitter.neutral(),
                                        robot.intakeGate.close(),
                                        new Instant(robot.intakeTilt::transfer)
                                ),
                                RobotStateHandler.CycleState.DRIVE_TO_SHOOT
                        ),
                        new Conditional(
                                robot.flywheel::isStopped,
                                new Instant(robot::shootNear),
                                Commands.NOOP
                        ),
                        new Wait(200),
                        new Instant(robot.intakeMotor::outtake)
                    )
            );
        }

        if (gamepad_2.dpad_up.isRisingEdge()) {
            robot.tableCompartments.intakeThread.reset();
            schedule(new Conditional(
                    () -> tsh.evaluate(RobotStateHandler.CycleState.SHOOT) && canShoot,
                    new Sequential(
                            new Instant(() -> canShoot = false),
                            tsh.runTransition(new Sequential(
                                    new Sequential(
                                            robot.shootAll(),
                                            new Instant(() -> canShoot = true)
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
            Pose reset = new ColoredDecodePose().getPose();
            robot.drivetrain.follower.setHeading(reset.getHeading());
        }

        if (gamepad_2.left_stick_y_button.isRisingEdge()) {
            if (gamepad2.left_stick_y < -0.3f) robot.intakeTilt.transfer();
            else if (gamepad2.left_stick_y > 0.3f) robot.intakeTilt.intake();
        }

        if (gamepad_2.right_stick_y_button.isRisingEdge()) {
            if (gamepad2.right_stick_y < -0.3f) robot.hood.finetuneFar(10/255d);
            else if (gamepad2.right_stick_y > 0.3f) robot.hood.finetuneFar(-10/255d);
        }

        if (gamepad_1.left_trigger_button.isRisingEdge()) {
            if (!gyroThread.isFar()) gyroThread.setState(TrackState.CLOSE_FOUR);
            else gyroThread.setState(TrackState.FAR_FOUR);
        }

        if (gamepad_1.right_trigger_button.isRisingEdge()) {
            if (!gyroThread.isFar()) gyroThread.setState(TrackState.CLOSE_THREE);
            else gyroThread.setState(TrackState.FAR_THREE);
        }

        if (gamepad_1.right_bumper.isRisingEdge()) {
            if (GyroThread.trackTurret) {
                if (!gyroThread.isFar()) gyroThread.setState(TrackState.CLOSE_ONE);
                else gyroThread.setState(TrackState.FAR_ONE);
            } else {
                schedule(tsh.task(() -> robot.turret.previous(), new int[]{1, 1, 0}));
            }
        }

        if (gamepad_1.left_bumper.isRisingEdge()) {
            if (GyroThread.trackTurret) {
                if (!gyroThread.isFar()) gyroThread.setState(TrackState.CLOSE_TWO);
                else gyroThread.setState(TrackState.FAR_TWO);
            } else {
                schedule(tsh.task(() -> robot.turret.next(), new int[]{1, 1, 0}));
            }
        }

        telemetry.addData("voltage", robot.voltageSensor.getVoltage());
    }
}