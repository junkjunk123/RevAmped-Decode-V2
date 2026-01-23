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
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.utils.AtomicReadOnce;
import org.firstinspires.ftc.teamcode.utils.GamepadEx;
import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;

@Config
@TeleOp(name = "DCTeleOp")
public class Tele extends OpModeCommand {
    private GamepadEx gamepad_1;
    private GamepadEx gamepad_2;
    private Robot robot;
    private TeleOpStateHandler tsh;
    public static AtomicReadOnce<Table.RelativeState> state;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        tsh = RobotStateHandler.createTeleOpStateHandler(robot);
        gamepad_1 = new GamepadEx(gamepad1);
        gamepad_2 = new GamepadEx(gamepad2);
        
        gamepad_1.left_trigger_button(f -> f.greaterThan(0.3f));
        gamepad_1.right_trigger_button(f -> f.greaterThan(0.3f));
        gamepad_2.left_trigger_button(f -> f.greaterThan(0.3f));
        gamepad_2.right_trigger_button(f -> f.greaterThan(0.3f));

        // Schedule robot update loop
        schedule(new Infinite(() -> {
            robot.update();
            robot.drivetrain.arcadeDrive(gamepad1);
        }));

        // Initialize robot
        schedule(new Sequential(
                new WaitUntil(() -> !opModeInInit()),
                robot.init(),
                tsh.runTransition(() -> {}, RobotStateHandler.CycleState.INTAKE)
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
    public void execute() {
        // Update switches
        gamepad_1.update();
        gamepad_2.update();

        // Schedule commands based on triggers
        if (gamepad_1.a.isRisingEdge()) schedule(new Instant(RobotStateHandler.CycleState.DriveToShoot::toggleDefault));
        if (gamepad_1.y.isRisingEdge()) schedule(new Instant(() -> tsh.setForce(!tsh.isForce())));
        if (gamepad_1.dpad_up.isRisingEdge()) schedule(tsh.setting(robot::shootFar));
        if (gamepad_1.dpad_down.isRisingEdge()) schedule(tsh.setting(robot::shootNear));
        if (gamepad_1.dpad_left.isRisingEdge()) schedule(tsh.setting(robot::shootMedium));

        if (gamepad_1.right_bumper.isRisingEdge()) schedule(tsh.task(robot.turret::next, new int[]{1, 1, 0}));
        if (gamepad_1.left_bumper.isRisingEdge()) schedule(tsh.task(robot.turret::previous, new int[]{1, 1, 0}));
        if (gamepad_1.b.isRisingEdge()) schedule(new Sequential(
                new WaitUntil(robot.turret.limitSwitch::state),
                new Instant(robot.turret::resetPosition)
        ));
        if (gamepad_1.x.isRisingEdge()) schedule(tsh.override(robot.popper.neutral(), RobotStateHandler.IntakeMessage.SORTING));

        if (gamepad_2.b.isRisingEdge() && (tsh.atState(RobotStateHandler.CycleState.DRIVE_TO_SHOOT) || !robot.intakeMotor.atPower(IntakeMotor.INTAKE))) {
            schedule(tsh.runTransition(new Parallel(
                    new Instant(() -> { robot.flywheel.stop(); robot.intakeMotor.intake(); }),
                    robot.table.reset(),
                    robot.popper.neutral()
            ), RobotStateHandler.CycleState.INTAKE));
        }

        if (gamepad_2.y.isRisingEdge()) schedule(robot.sort());
        if (gamepad_2.right_trigger_button.isRisingEdge()) schedule(new Instant(robot.intakeMotor::outtake));
        if (gamepad_2.a.isRisingEdge()) schedule(new Instant(robot.intakeMotor::stop));
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
                            new Instant(() -> {
                                //robot.tableCompartments.intakeThread.updateColors();
                                robot.intakeMotor.stop();
                            }),
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
                            tsh.runTransition(new Sequential(
                                    robot.shootAll(),
                                    robot.resetAfterShooting()
                            ), RobotStateHandler.CycleState.INTAKE)
                    ),
                    Commands.NOOP
            ));
        }

        if (gamepad_2.dpad_down.isRisingEdge()) {
            schedule(new Sequential(
                    tsh.runTransition(() -> {}, RobotStateHandler.CycleState.SHOOT),
                    tsh.runTransition(new Sequential(
                            robot.shootAll(175),
                            robot.resetAfterShooting()
                    ), RobotStateHandler.CycleState.INTAKE)
            ));
        }


        // Telemetry
        telemetry.addData("currentState", tsh.currentState());
        telemetry.addData("tableMoving", robot.table.pendingState() != robot.table.getState());
        telemetry.addData("tableVel", robot.table.getEncoder().getVelocity());
        telemetry.update();
    }

    @Override
    public void end() {
        //Turret.startPos = 0;
    }
}