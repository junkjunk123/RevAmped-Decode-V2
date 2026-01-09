package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.ivy.bindings.Bindings.bind;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.TeleOpStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Popper;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.TrackingThread;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.utils.GamepadEx;

@Config
@TeleOp(name = "DCTeleOp")
public class Tele extends OpModeCommand {
    private Robot robot;
    private TeleOpStateHandler tsh;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        tsh = RobotStateHandler.createTeleOpStateHandler(robot);
    }

    @Override
    public void onStart() {
        // Schedule robot update loop
        schedule(infinite(() -> {
            robot.update();
            robot.drivetrain.arcadeDrive(gamepad1);

            if (gamepad1.right_trigger > 0.3) schedule(tsh.task(
                    () -> robot.turret.setTargetPosition(robot.turret.getTargetPosition() + (int) (20 * gamepad1.right_trigger)),
                    new int[]{1, 1, 0}
            ));

            if (gamepad1.left_trigger > 0.3) schedule(tsh.task(
                    () -> robot.turret.setTargetPosition(robot.turret.getTargetPosition() - (int) (20 * gamepad1.left_trigger)),
                    new int[]{1, 1, 0}
            ));

            telemetry.addData("cycleState", tsh.currentState());
            telemetry.addData("force",tsh.isForce());
            telemetry.update();
        }));

        // Initialize robot
        schedule(robot.init());

        bind(() -> gamepad1.a)
                .rise(instant(() -> TrackingThread.trackTurret = true));

        bind(() -> gamepad1.y)
                .rise(instant(() -> tsh.setForce(!tsh.isForce())));

        bind(() -> gamepad1.dpad_up)
                .rise(tsh.task(robot::shootFar, new int[]{1, 1, 0}));

        bind(() -> gamepad1.dpad_down)
                .rise(tsh.task(robot::shootNear, new int[]{1, 1, 0}));

        bind(() -> gamepad1.dpad_left)
                .rise(tsh.task(robot::shootMedium, new int[]{1, 1, 0}));

        bind(() -> gamepad1.right_bumper)
                .rise(tsh.task(robot.turret::next, new int[]{1, 1, 0}));

        bind(() -> gamepad1.left_bumper)
                .rise(tsh.task(robot.turret::previous, new int[]{1, 1, 0}));

        bind(() -> gamepad1.b)
                .rise(race(
                        sequential(
                                waitUntil(robot.turret.limitSwitch::state),
                                instant(robot.turret::resetPosition)
                        ),
                        waitMs(750.0)
                ));

        bind(() -> gamepad1.x)
                .rise(tsh.override(
                        robot.popper.neutral(),
                        RobotStateHandler.IntakeMessage.SORTING
                ));

        bind(() -> gamepad2.b)
                .rise(conditional(
                        () -> tsh.atState(RobotStateHandler.CycleState.DRIVE_TO_SHOOT)
                                        || !robot.intakeMotor.atPower(IntakeMotor.INTAKE),
                        tsh.runTransition(
                                parallel(
                                        instant(() -> {
                                            robot.flywheel.stop();
                                            robot.intakeMotor.intake();
                                        }),
                                        robot.table.reset(),
                                        robot.popper.neutral()
                                ),
                                RobotStateHandler.CycleState.INTAKE
                        ),
                        Command.NOOP
                ));

        bind(() -> gamepad2.y)
                .rise(robot.sort());

        bind(() -> gamepad2.right_trigger > 0.3)
                .rise(instant(robot.intakeMotor::outtake));

        bind(() -> gamepad2.a)
                .rise(instant(robot.intakeMotor::stop));

        bind(() -> gamepad2.right_bumper)
                .rise(tsh.task(
                        sequential(
                                instant(() ->
                                        robot.setRobotState(RobotStateHandler.IntakeMessage.SORTING)
                                ),
                                robot.table.previous()
                        ),
                        new int[]{1, 0, 0}
                ));

        bind(() -> gamepad2.left_bumper)
                .rise(tsh.task(
                        sequential(
                                instant(() ->
                                        robot.setRobotState(RobotStateHandler.IntakeMessage.SORTING)
                                ),
                                robot.table.next()
                        ),
                        new int[]{1, 0, 0}
                ));

        bind(() -> gamepad2.x)
                .rise(conditional(
                        () ->
                                tsh.atState(RobotStateHandler.CycleState.INTAKE)
                                        && robot.popper.atState(Popper.PopperState.NEUTRAL),
                        tsh.runTransition(
                                sequential(
                                        instant(robot.intakeMotor::stop),
                                        robot.popper.pop()
                                ),
                                RobotStateHandler.CycleState.DRIVE_TO_SHOOT
                        ),
                        Command.NOOP
                ));

        bind(() -> gamepad2.dpad_up)
                .rise(conditional(
                        () -> tsh.evaluate(RobotStateHandler.CycleState.SHOOT),
                        sequential(
                                tsh.runTransition(() -> {}, RobotStateHandler.CycleState.SHOOT),
                                tsh.runTransition(
                                        sequential(
                                                robot.shootAll(),
                                                robot.resetAfterShooting()
                                        ),
                                        RobotStateHandler.CycleState.INTAKE
                                )
                        ),
                        Command.NOOP
                ));

        bind(() -> gamepad2.dpad_down)
                .rise(conditional(
                        () -> tsh.evaluate(RobotStateHandler.CycleState.SHOOT),
                        sequential(
                                tsh.runTransition(() -> {}, RobotStateHandler.CycleState.SHOOT),
                                tsh.runTransition(
                                        sequential(
                                                robot.shootAll(175),
                                                robot.resetAfterShooting()
                                        ),
                                        RobotStateHandler.CycleState.INTAKE
                                )
                        ),
                        Command.NOOP
                ));
    }

    @Override
    public void end() {
        Turret.startPos = 0;
    }
}