package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.TeleOpStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeThread;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Popper;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.mechanisms.lift.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.GyroThread;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurretState;
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
    private boolean gyroTrack;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Globals.telemetry = telemetry;
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
        robot.splitter.setUseSplitter(false);
        
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
            if (gyroTrack) gyroThread.update();
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
                                robot.popper.moveToNeutral(),
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
                                                robot.splitter.isUseSplitter() ? new Parallel(
                                                        robot.popper.neutral(),
                                                        new Sequential(
                                                                new Wait(300),
                                                                robot.splitter.activate()
                                                        )
                                                ) : robot.popper.neutral()
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
        robot.tableCompartments.intakeThread.reset();
        sendLEDs();
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
            sendLEDs();
        }

        if (gamepad_1.y.isRisingEdge()) {
            if (!robot.drivetrain.isHoldingPose()) schedule(robot.drivetrain.holdPose());
            else robot.drivetrain.stopHoldPose();
        }

        if (gamepad_1.dpad_up.isRisingEdge()){
            if (!gyroThread.isFar()) gyroThread.setState(TrackState.CLOSE_THREE);
            else gyroThread.setState(TrackState.FAR_THREE);
            schedule(tsh.setting(robot::shootCorner), new Instant(gyroThread::close));
        }

        if (gamepad_1.dpad_down.isRisingEdge()) schedule(tsh.setting(() -> {
            robot.shootNear();
            gyroThread.close();
        }), new Instant(gyroThread::close));

        if (gamepad_1.dpad_left.isRisingEdge()) schedule(tsh.setting(() -> {
            robot.shootMedium();
            gyroThread.close();
        }));

        if (gamepad_1.dpad_right.isRisingEdge()) schedule(tsh.setting(() -> {
            robot.shootFar();
            gyroThread.close();
        }));

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

        if (gamepad_1.back.isRisingEdge()){
            schedule(
                robot.lift.lift()
            );
        }

        if (gamepad_2.b.isRisingEdge() && (tsh.atState(RobotStateHandler.CycleState.DRIVE_TO_SHOOT) || !robot.intakeMotor.atState(IntakeMotor.IntakeState.INTAKE))) {
            schedule(tsh.runTransition(new Parallel(
                    new Instant(() -> {
                        robot.flywheel.stop();
                        robot.intakeTilt.intake();
                        robot.feederWheel.intakeState();
                    }),
                    new Sequential(
                            robot.resetTableTeleOp(),
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

        /*
        if (!transfer && IntakeThread.useSensors && tsh.atState(RobotStateHandler.CycleState.INTAKE) && robot.intakeMotor.getPower() > 0.2) {
            if (robot.tableCompartments.intakeThread.hasThree) {
                robot.tableCompartments.populate();
                transfer = true;
            }
        }
         */

        if (transfer || (gamepad_2.x.isRisingEdge() && tsh.atState(RobotStateHandler.CycleState.INTAKE) && robot.popper.atState(Popper.PopperState.NEUTRAL))) {
            transfer = false;
            gyroTrack = true;
            RobotStateHandler.CycleState.INTAKE.update = false;
            schedule(new Sequential(
                        new Instant(() -> robot.intakeTilt.transfer()),
                        tsh.runTransition(
                                robot.splitter.isUseSplitter() ? new Parallel(
                                        new Sequential(
                                                new Wait(50),
                                                robot.popper.pop()
                                        ),
                                        new Sequential(
                                                new Wait(150),
                                                robot.splitter.neutral()
                                        )
                                ) : new Sequential(
                                        new Wait(50),
                                        robot.popper.pop()
                                ),
                                RobotStateHandler.CycleState.DRIVE_TO_SHOOT
                        ),
                        new Instant(() -> robot.feederWheel.start()),
                        new Sequential(
                            new Instant(robot.intakeMotor::outtakeSlow),
                            new Wait(50),
                            robot.intakeGate.close(),
                            new Instant(robot.intakeMotor::stop)
                        )
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
                                    new Parallel(
                                            robot.resetAfterShooting(),
                                            new Instant(() -> {
                                                gyroTrack = false;
                                                robot.turret.move(new ServoTurretState.Custom(ServoTurret.REST));
                                            })
                                    )
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

        if (gamepad_2.dpad_right.isRisingEdge()) {
            robot.drivetrain.follower.setHeading(Math.toRadians(270));
        }

        if (gamepad_2.dpad_left.isRisingEdge()) {
            schedule(tsh.task(new Parallel(
                    new Sequential(
                            new Instant(() -> robot.intakeMotor.stop()),
                            new Wait(200),
                            new Instant(robot.intakeMotor::intake)
                    ),
                    robot.popper.neutral(),
                    new Sequential(
                            new Wait(350),
                            new Instant(() -> transfer = true)
                    ),
                    new Conditional(
                            robot.flywheel::isStopped,
                            new Instant(robot::shootNear),
                            Commands.NOOP
                    )
            ), new int[] {1, 0, 0}));
        }

        if (gamepad_2.left_stick_y_button.isRisingEdge()) {
            if (gamepad2.left_stick_y < -0.3f) robot.intakeTilt.transfer();
            else if (gamepad2.left_stick_y > 0.3f) robot.intakeTilt.intake();
        }

        if (gamepad2.left_stick_x > 0.3f){
            robot.intakeTilt.gateIntake();
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
        telemetry.addData("popper_state", robot.popper.getState());
        telemetry.addData("robot_state", robot.getRobotState());
        telemetry.addData("hasThree", robot.tableCompartments.intakeThread.hasThree);
        telemetry.addData("encoderPos", robot.table.getEncoder().getPosition());
        telemetry.addData("encoderVel", robot.table.getEncoder().getVelocity());
    }

    private void sendLEDs() {
        if (GyroThread.trackTurret) gamepad1.setLedColor(0,1,0, Gamepad.LED_DURATION_CONTINUOUS);
        else gamepad1.setLedColor(1,0,0, Gamepad.LED_DURATION_CONTINUOUS);
    }
}