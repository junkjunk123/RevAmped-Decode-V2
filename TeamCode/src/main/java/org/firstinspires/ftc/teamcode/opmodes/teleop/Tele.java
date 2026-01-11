package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.TeleOpStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Popper;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.TrackingThread;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.utils.AtomicReadOnce;
import org.firstinspires.ftc.teamcode.utils.BooleanSwitch;
import org.firstinspires.ftc.teamcode.utils.GamepadEx;
import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;

@Config
@TeleOp(name = "DCTeleOp")
public class Tele extends OpModeCommand {
    private BooleanSwitch a, y, dpadUp1, dpadDown1, dpadLeft1, rightTrigger1, leftTrigger1, rightBumper1,
            leftBumper1, b1, x1, b2, y2, dpadRight2, rightTrigger2, rightBumper2, leftBumper2,
            x2, dpadUp2, dpadDown2;
    private Robot robot;
    private TeleOpStateHandler tsh;
    public static AtomicReadOnce<Table.RelativeState> state;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        tsh = RobotStateHandler.createTeleOpStateHandler(robot);
        GamepadEx gamepad_1 = new GamepadEx(gamepad1);
        GamepadEx gamepad_2 = new GamepadEx(gamepad2);

        // Initialize switches
        a = gamepad_1.a.risingEdge();
        y = gamepad_1.y.risingEdge();
        dpadUp1 = gamepad_1.dpad_up.risingEdge();
        dpadDown1 = gamepad_1.dpad_down.risingEdge();
        dpadLeft1 = gamepad_1.dpad_left.risingEdge();
        rightTrigger1 = gamepad_1.right_trigger.greaterThan(0.3f).risingEdge();
        leftTrigger1 = gamepad_1.left_trigger.greaterThan(0.3f).risingEdge();
        rightBumper1 = gamepad_1.right_bumper.risingEdge();
        leftBumper1 = gamepad_1.left_bumper.risingEdge();
        b1 = gamepad_1.b.risingEdge();
        x1 = gamepad_1.x.risingEdge();

        b2 = gamepad_2.b.risingEdge();
        y2 = gamepad_2.y.risingEdge();
        dpadRight2 = gamepad_2.dpad_right.risingEdge();
        rightTrigger2 = gamepad_2.right_trigger.greaterThan(0.3f).risingEdge();
        rightBumper2 = gamepad_2.right_bumper.risingEdge();
        leftBumper2 = gamepad_2.left_bumper.risingEdge();
        x2 = gamepad_2.x.risingEdge();
        dpadUp2 = gamepad_2.dpad_up.risingEdge();
        dpadDown2 = gamepad_2.dpad_down.risingEdge();

        // Schedule robot update loop
        schedule(new Infinite(() -> {
            robot.update();
            robot.drivetrain.arcadeDrive(gamepad1);
        }));

        // Initialize robot
        schedule(new Sequential(
                new WaitUntil(() -> !opModeInInit()),
                robot.init()
        ));
    }

    @Override
    public void execute() {
        // Update switches
        a.update(); y.update(); dpadUp1.update(); dpadDown1.update(); dpadLeft1.update();
        rightTrigger1.update(); leftTrigger1.update(); rightBumper1.update(); leftBumper1.update();
        b1.update(); x1.update();

        b2.update(); y2.update(); dpadRight2.update(); rightTrigger2.update(); rightBumper2.update();
        leftBumper2.update(); x2.update(); dpadUp2.update(); dpadDown2.update();

        // Schedule commands based on triggers
        if (a.isTrue()) schedule(new Instant(() -> TrackingThread.trackTurret = true));
        if (y.isTrue()) schedule(new Instant(() -> tsh.setForce(!tsh.isForce())));
        if (dpadUp1.isTrue()) schedule(tsh.setting(robot::shootFar));
        if (dpadDown1.isTrue()) schedule(tsh.setting(robot::shootNear));
        if (dpadLeft1.isTrue()) schedule(tsh.setting(robot::shootMedium));
        if (rightTrigger1.isTrue()) schedule(tsh.task(
                () -> robot.turret.runToPos(robot.turret.getTargetPosition() + (int) (20 * gamepad1.right_trigger)),
                new int[]{1, 1, 0}
        ));
        if (leftTrigger1.isTrue()) schedule(tsh.task(
                () -> robot.turret.runToPos(robot.turret.getTargetPosition() - (int) (20 * gamepad1.left_trigger)),
                new int[]{1, 1, 0}
        ));
        if (rightBumper1.isTrue()) schedule(tsh.task(robot.turret::next, new int[]{1, 1, 0}));
        if (leftBumper1.isTrue()) schedule(tsh.task(robot.turret::previous, new int[]{1, 1, 0}));
        if (b1.isTrue()) schedule(new Sequential(
                new WaitUntil(robot.turret.limitSwitch::state),
                new Instant(robot.turret::resetPosition)
        ));
        if (x1.isTrue()) schedule(tsh.override(robot.popper.neutral(), RobotStateHandler.IntakeMessage.SORTING));

        if (b2.isTrue() && (tsh.atState(RobotStateHandler.CycleState.DRIVE_TO_SHOOT) || !robot.intakeMotor.atPower(IntakeMotor.INTAKE))) {
            schedule(tsh.runTransition(new Parallel(
                    new Instant(() -> { robot.flywheel.stop(); robot.intakeMotor.intake(); }),
                    robot.table.reset(),
                    robot.popper.neutral()
            ), RobotStateHandler.CycleState.INTAKE));
        }

        if (y2.isTrue()) schedule(robot.sort());
        if (dpadRight2.isTrue()) schedule(new Instant(robot.intakeMotor::outtake));
        if (rightTrigger2.isTrue()) schedule(new Instant(robot.intakeMotor::stop));
        if (rightBumper2.isTrue()) schedule(tsh.task(
                new Sequential(
                        new Instant(() -> robot.setRobotState(RobotStateHandler.IntakeMessage.SORTING)),
                        robot.table.previous()
                ), new int[]{1, 0, 0}
        ));

        if (leftBumper2.isTrue()) schedule(tsh.task(
                new Sequential(
                        new Instant(() -> robot.setRobotState(RobotStateHandler.IntakeMessage.SORTING)),
                        robot.table.next()
                ), new int[]{1, 0, 0}
        ));

        if (x2.isTrue() && tsh.atState(RobotStateHandler.CycleState.INTAKE) && robot.popper.atState(Popper.PopperState.NEUTRAL)) {
            schedule(tsh.runTransition(
                    new Sequential(
                            new Instant(() -> {
                                //robot.tableCompartments.intakeThread.updateColors();
                                robot.intakeMotor.stop();
                            }),
                            robot.popper.pop()
                    ),
                    RobotStateHandler.CycleState.DRIVE_TO_SHOOT
            ));
        }

        if (dpadUp2.isTrue()) {
            schedule(new Conditional(
                    () -> tsh.evaluate(RobotStateHandler.CycleState.SHOOT),
                    new Sequential(
                            tsh.runTransition(() -> {}, RobotStateHandler.CycleState.SHOOT),
                            tsh.runTransition(new Sequential(
                                    robot.shootAll(),
                                    robot.resetAfterShooting()
                            ), RobotStateHandler.CycleState.INTAKE)
                    ),
                    Commands.NOOP
            ));
        }

        if (dpadDown2.isTrue()) {
            schedule(new Sequential(
                    tsh.runTransition(() -> {}, RobotStateHandler.CycleState.SHOOT),
                    tsh.runTransition(new Sequential(
                            robot.shootAll(175),
                            robot.resetAfterShooting()
                    ), RobotStateHandler.CycleState.INTAKE)
            ));
        }

        // Telemetry
        telemetry.addData("intake -> drive", tsh.evaluate(RobotStateHandler.CycleState.DRIVE_TO_SHOOT));
        telemetry.addData("adj", tsh.getAdj());
        telemetry.addData("evaluate", tsh.evaluate(RobotStateHandler.CycleState.SHOOT));
        telemetry.addData("turret", Robot.INSTANCE.turret.getTargetPosition());
        telemetry.addData("popper", Robot.INSTANCE.popper.atState(Popper.PopperState.NEUTRAL));
        telemetry.addData("cycleState", tsh.currentState());
        telemetry.addData("table", robot.table.getState());
        telemetry.addData("state", state.hasBeenRead() ? state.read() : "null");
        telemetry.update();
    }

    @Override
    public void end() {
        Turret.startPos = 0;
    }
}