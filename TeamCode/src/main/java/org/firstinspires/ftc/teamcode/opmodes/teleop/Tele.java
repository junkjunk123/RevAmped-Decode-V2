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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.math.projectile.ShooterMath;
import org.firstinspires.ftc.teamcode.math.projectile.SimpleShooterMath;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.TeleOpStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Popper;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.TrackingThread;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.pedro.ColoredDecodePose;
import org.firstinspires.ftc.teamcode.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.AtomicReadOnce;
import org.firstinspires.ftc.teamcode.utils.GamepadEx;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RandomizationState;
import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.commands.Lazy;
import org.firstinspires.ftc.teamcode.utils.logging.DecodeLogger;
import org.firstinspires.ftc.teamcode.utils.prompter.OptionPrompt;
import org.firstinspires.ftc.teamcode.utils.prompter.Prompter;
import org.firstinspires.ftc.teamcode.utils.prompter.StatePrompt;

@Config
@TeleOp(name = "DCTeleOp")
public class Tele extends OpModeCommand {
    private static final double TELEMETRY_PERIOD_MS = 100.0;
    private GamepadEx gamepad_1;
    private GamepadEx gamepad_2;
    private Robot robot;
    private TeleOpStateHandler tsh;
    private Prompter prompter;
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private double lastLoopMs;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        tsh = RobotStateHandler.createTeleOpStateHandler(robot);
        gamepad_1 = new GamepadEx(gamepad1);
        gamepad_2 = new GamepadEx(gamepad2);
        prompter = new Prompter(this, gamepad_1)
                .prompt("motif", new StatePrompt<>("Select the motif pattern", RandomizationState.class))
                .thenDisplay("Good luck! We're rooting for you. --- Havish & Eric");
        
        gamepad_1.left_trigger_button(f -> f.greaterThan(0.3f));
        gamepad_1.right_trigger_button(f -> f.greaterThan(0.3f));
        gamepad_2.left_trigger_button(f -> f.greaterThan(0.3f));
        gamepad_2.right_trigger_button(f -> f.greaterThan(0.3f));
        //robot.drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Schedule robot update loop
        schedule(new Infinite(() -> {
            robot.update();
            robot.drivetrain.arcadeDrive(gamepad1);
        }));

        // Initialize robot
        schedule(new Sequential(
                new WaitUntil(() -> !opModeInInit()),
                new Instant(robot::initialize),
                new Instant(() -> Globals.randomizationState = prompter.getOrDefault("motif", Globals.randomizationState)),
                tsh.runTransition(() -> {}, RobotStateHandler.CycleState.SHOOT),
                tsh.runTransition(
                    new Sequential(
                        robot.shootAll(),
                        robot.resetAfterShooting()
                    ), RobotStateHandler.CycleState.INTAKE)
                /* ,
                tsh.runTransition(
                        new Sequential(robot.popper.pop()),
                        RobotStateHandler.CycleState.DRIVE_TO_SHOOT
                ),
                tsh.runTransition(() -> {}, RobotStateHandler.CycleState.SHOOT),
                tsh.runTransition(new Sequential(
                        robot.shootAll(),
                        robot.resetAfterShooting()
                ), RobotStateHandler.CycleState.INTAKE)
                 */
        ));
    }

    @Override
    public void initializeLoop() {
        telemetry.addData("alliance", Globals.allianceColor);
        prompter.run();
    }

    @Override
    public void execute() {
        long loopStartNanos = System.nanoTime();
//        if (!teleReset){
//
//            teleReset = true;
//        }
        // Update switches
        gamepad_1.update();
        gamepad_2.update();

        // Schedule commands based on triggers
        if (gamepad_1.a.isRisingEdge()) {
            TrackingThread.trackTurret = !TrackingThread.trackTurret;
            TrackingThread.trackHood = !TrackingThread.trackHood;
            DecodeLogger.get().info("tracking", "TRACKING_TOGGLE",
                    "trackTurret", TrackingThread.trackTurret,
                    "trackHood", TrackingThread.trackHood);
        }

        if (gamepad_1.y.isRisingEdge()) {
            tsh.setForce(!tsh.isForce());
        }

        if (gamepad_1.dpad_up.isRisingEdge()) schedule(tsh.setting(robot::shootFar));
        if (gamepad_1.dpad_down.isRisingEdge()) schedule(tsh.setting(robot::shootNear));
        if (gamepad_1.dpad_left.isRisingEdge()) schedule(tsh.setting(robot::shootMedium));

        if (gamepad_1.right_bumper.isRisingEdge()) {
            if (!TrackingThread.trackTurret) schedule(tsh.task(robot.turret::next, new int[]{1, 1, 0}));
            else {
                SimpleShooterMath.APRIL_TAG_POSE_BLUE = SimpleShooterMath.APRIL_TAG_POSE_BLUE.plus(new Pose(3, 3));
                SimpleShooterMath.APRIL_TAG_POSE_RED = SimpleShooterMath.APRIL_TAG_POSE_RED.plus(new Pose(3, -3));
            }
        }

        if (gamepad_1.left_bumper.isRisingEdge()) {
            if (!TrackingThread.trackTurret) schedule(tsh.task(robot.turret::previous, new int[]{1, 1, 0}));
            else {
                SimpleShooterMath.APRIL_TAG_POSE_BLUE = SimpleShooterMath.APRIL_TAG_POSE_BLUE.plus(new Pose(-3, -3));
                SimpleShooterMath.APRIL_TAG_POSE_RED = SimpleShooterMath.APRIL_TAG_POSE_RED.plus(new Pose(-3, 3));
            }
        }

        if (gamepad_1.b.isRisingEdge()) schedule(new Sequential(
                new WaitUntil(robot.turret.limitSwitch::state),
                new Instant(robot.turret::resetPosition)
        ));
        if (gamepad_1.x.isRisingEdge()) schedule(tsh.override(robot.popper.neutral(), RobotStateHandler.IntakeMessage.SORTING));

        if (gamepad_1.right_trigger_button.isTrue())
            robot.turret.finetune((int) (gamepad1.right_trigger * 20));

        if (gamepad_1.left_trigger_button.isTrue())
            robot.turret.finetune(-(int) (gamepad1.left_trigger * 20));

        if (gamepad_2.b.isRisingEdge() && (tsh.atState(RobotStateHandler.CycleState.DRIVE_TO_SHOOT) || !robot.intakeMotor.atState(IntakeMotor.IntakeState.INTAKE))) {
            schedule(tsh.runTransition(new Parallel(
                    new Instant(robot.flywheel::stop),
                    robot.resetTable(),
                    new Lazy(() -> {
                        if (TrackingThread.trackTurret) return robot.turret.resetTurret();
                        else return Commands.NOOP;
                    })
            ), RobotStateHandler.CycleState.INTAKE));
        }

        if (gamepad_2.y.isRisingEdge()) schedule(robot.sort());
        if (gamepad_2.right_trigger_button.isRisingEdge()) robot.intakeMotor.outtake();
        if (gamepad_2.a.isRisingEdge()) robot.intakeMotor.stop();
        if (gamepad_2.right_bumper.isRisingEdge()) schedule(tsh.task(
                new Sequential(
                        new Instant(() -> robot.setRobotState(RobotStateHandler.IntakeMessage.SORTING)),
                        robot.table.previous()
                ), new int[]{1, 0, 0}
        ));

        if (gamepad_2.left_bumper.isRisingEdge()) schedule(tsh.task(
                new Sequential(
                        new Instant(() -> robot.setRobotState(RobotStateHandler.IntakeMessage.SORTING)),
                        robot.table.next()
                ), new int[]{1, 0, 0}
        ));

        if (gamepad_2.x.isRisingEdge() && tsh.atState(RobotStateHandler.CycleState.INTAKE) && robot.popper.atState(Popper.PopperState.NEUTRAL)) {
            schedule(tsh.runTransition(
                    new Parallel(
                            new Instant(() -> robot.intakeMotor.outtake()),
                            //stops intake after 500 ms (doesn't do anything if the robot is trying to shoot)
                            new Race(
                                new WaitUntil(() -> tsh.atState(RobotStateHandler.CycleState.SHOOT)),
                                new Sequential(
                                        new Wait(500),
                                        new Instant(() -> robot.intakeMotor.stop())
                                )
                            ),
                            //new Instant(() -> {robot.intakeMotor.outtake();}),
                            robot.popper.pop(),
                            new Conditional(
                                    robot.flywheel::isStopped,
                                    new Instant(robot::shootMedium),
                                    Commands.NOOP
                            )
                    ),
                    RobotStateHandler.CycleState.DRIVE_TO_SHOOT
            ));
        }

        if (gamepad_2.dpad_up.isRisingEdge()) {
            schedule(new Conditional(
                    () -> tsh.evaluate(RobotStateHandler.CycleState.SHOOT),
                    new Sequential(
                            tsh.runTransition(() -> {}, RobotStateHandler.CycleState.SHOOT),
                            tsh.runTransition(new Conditional(
                                    () -> robot.hood.getCurrentState() == Hood.HoodState.FAR,
                                    //is shooting far
                                    new Sequential(
                                    robot.shootAll(10),
                                    robot.resetAfterShooting()
                                    ),
                                    //not shooting far
                                    new Sequential(
                                            robot.shootAll(),
                                            robot.resetAfterShooting())
                                    ), RobotStateHandler.CycleState.INTAKE)
                    ),
                    Commands.NOOP
            ));
        }

        if (gamepad_2.dpad_down.isRisingEdge()) {
            schedule(new Sequential(
                    tsh.runTransition(() -> {}, RobotStateHandler.CycleState.SHOOT),
                    tsh.runTransition(new Sequential(
                            robot.shootAll(Table.SLOW_SHOOT_DELAY),
                            robot.resetAfterShooting()
                    ), RobotStateHandler.CycleState.INTAKE)
            ));
        }

        if (gamepad_2.left_trigger_button.isRisingEdge()) {
            robot.turret.resetPosition();
        }

        if (gamepad_2.dpad_left.isRisingEdge()) {
            schedule(tsh.task(robot.popper.block(), RobotStateHandler.CycleState.INTAKE));
        } else if (gamepad_2.dpad_left.isFallingEdge()) {
            schedule(tsh.task(robot.popper.neutral(), RobotStateHandler.CycleState.INTAKE));
        }

        if (gamepad_2.dpad_right.isRisingEdge()) {
            robot.drivetrain.follower.setPose(new ColoredDecodePose(72, 130.5, Math.PI).getPose());
        }

        lastLoopMs = (System.nanoTime() - loopStartNanos) / 1_000_000.0;
        if (telemetryTimer.milliseconds() >= TELEMETRY_PERIOD_MS) {
            telemetry.addData("alliance", Globals.allianceColor);
            telemetry.addData("cycleState", tsh.currentState().toString());
            telemetry.addData("flywheelTarget", "%.1f", robot.flywheel.getTargetVelocity());
            telemetry.addData("flywheelVel", "%.1f", robot.flywheel.getVelocityImperial());
            telemetry.addData("flywheelError", "%.1f", robot.flywheel.getError());
            telemetry.addData("turretTarget", robot.turret.getTargetPosition());
            telemetry.addData("turretPos", robot.turret.getPosition());
            telemetry.addData("hoodState", robot.hood.getState());
            telemetry.addData("popperState", robot.popper.getState());
            telemetry.addData("intakeState", robot.intakeMotor.getState());
            telemetry.addData("loopMsExecute", "%.2f", lastLoopMs);
            telemetry.update();
            telemetryTimer.reset();
        }
    }

    @Override
    public void end() {
        //Turret.startPos = 0;
    }
}
