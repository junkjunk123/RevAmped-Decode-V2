package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.TeleOpStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeThread;
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
    private boolean canShoot = true;
    private GyroThread gyroThread;
    private boolean gyroTrack;
    private boolean firstTrack = true;
    private boolean transfer = false;
    private final ColoredDecodePose resetPose = new ColoredDecodePose(0, 0, Math.PI);

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

        tsh.setState(RobotStateHandler.CycleState.INTAKE);

        // Initialize robot
        schedule(new Sequential(
                new WaitUntil(() -> !opModeInInit()),
                new Instant(robot::initialize),
                new Wait(500)
                )
        );
    }

    @Override
    public void initializeLoop() {
        telemetry.addData("alliance", Globals.allianceColor);
        prompter.run();
    }
    @Override
    public void onStart() {
        robot.drivetrain.follower.setPose(Drivetrain.startPose);
        robot.drivetrain.follower.update();
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
            gamepad_1.rumble(500);
        }

        if (gamepad_1.dpad_up.isRisingEdge()){
            if (!gyroThread.isFar()) gyroThread.setState(TrackState.CLOSE_THREE);
            else gyroThread.setState(TrackState.FAR_THREE);
            schedule(tsh.setting(robot::shootCorner), new Instant(gyroThread::close));
        }

        if (gamepad_1.dpad_down.isRisingEdge()) schedule(tsh.setting(() -> {
            robot.shootNear();
            if (firstTrack) {
                gyroThread.setState(TrackState.CLOSE_ONE);
                firstTrack = false;
            } else {
                gyroThread.close();
            }
        }));

        if (gamepad_1.dpad_left.isRisingEdge()) schedule(tsh.setting(() -> {
            robot.shootMedium();

            if (firstTrack) {
                gyroThread.setState(TrackState.CLOSE_ONE);
                firstTrack = false;
            } else {
                gyroThread.close();
            }
        }));

        if (gamepad_1.dpad_right.isRisingEdge()) schedule(tsh.setting(() -> {
            robot.shootFar();
            if (firstTrack) {
                gyroThread.setState(TrackState.FAR_ONE);
                firstTrack = false;
            } else {
                gyroThread.far();
            }
        }));

        if (gamepad_1.start.isRisingEdge()){
            schedule(robot.turret.resetTurret());
        }

        if (gamepad_2.b.isRisingEdge() && (tsh.atState(RobotStateHandler.CycleState.DRIVE_TO_SHOOT) || !robot.intakeMotor.atState(IntakeMotor.IntakeState.INTAKE))) {
            schedule(tsh.runTransition(
                    new Instant(() -> {
                        robot.flywheel.stop();
                        robot.intakeTilt.intake();
                        robot.feederWheel.intakeState();
                    }
            ), RobotStateHandler.CycleState.INTAKE));
        }
        if (gamepad_2.right_trigger_button.isRisingEdge()) {
            if (robot.intakeMotor.atState(IntakeMotor.IntakeState.OUTTAKE)) robot.intakeMotor.stop();
            else robot.intakeMotor.outtake();
            robot.intakeTilt.transfer();
        }

        if (!transfer && IntakeThread.useSensors && tsh.atState(RobotStateHandler.CycleState.INTAKE) && robot.intakeMotor.getPower() > 0.2) {
            if (RobotStateHandler.CycleState.INTAKE.update) { //&& has three
                transfer = true;
            }
        }

        if (transfer || (gamepad_2.x.isRisingEdge() && tsh.atState(RobotStateHandler.CycleState.INTAKE))) {
            gyroTrack = true;
            transfer = false;
            RobotStateHandler.CycleState.INTAKE.update = false;
            schedule(tsh.runTransition(new Sequential(
                    new Parallel(
                            new Instant(robot.intakeTilt::transfer),
                            new Instant(robot.intakeMotor::outtake),
                            new Conditional(
                                    robot.flywheel::isStopped,
                                    new Instant(robot::shootNear),
                                    Commands.NOOP
                            )
                    ),
                    new Instant(() -> RobotStateHandler.CycleState.INTAKE.update = true)
                ), RobotStateHandler.CycleState.DRIVE_TO_SHOOT)
            );
        }

        if (gamepad_2.dpad_up.isRisingEdge()) {
            schedule(new Conditional(
                    () -> tsh.evaluate(RobotStateHandler.CycleState.SHOOT) && canShoot,
                    new Sequential(
                            new Instant(() -> canShoot = false),
                            tsh.runTransition(new Sequential(
                                    new Sequential(
                                            robot.autoShoot(),
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

        if (gamepad_2.dpad_right.isRisingEdge()) {
            robot.drivetrain.follower.setHeading(resetPose.getHeading());
            GyroThread.NEUTRAL_OFFSET = 0;
        }

        if (gamepad_2.right_bumper.isRisingEdge()) {
            schedule(new Instant(() -> GyroThread.NEUTRAL_OFFSET -= 1/255d));
        }

        else if (gamepad_2.left_bumper.isRisingEdge()) {
            schedule(new Instant(() -> GyroThread.NEUTRAL_OFFSET += 1/255d));
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
    }

    private void sendLEDs() {
        if (GyroThread.trackTurret) gamepad1.setLedColor(0,1,0, Gamepad.LED_DURATION_CONTINUOUS);
        else gamepad1.setLedColor(1,0,0, Gamepad.LED_DURATION_CONTINUOUS);
    }
}