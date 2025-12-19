package org.firstinspires.ftc.teamcode.revamped.opmodes.teleop;

import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Sequential;

import org.firstinspires.ftc.teamcode.revamped.Robot;
import org.firstinspires.ftc.teamcode.revamped.RobotStateHandler;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.TeleOpStateHandler;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.intake.Popper;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.shooter.TrackingThread;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.revamped.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.revamped.utils.Commands.ButtonMapper;
import org.firstinspires.ftc.teamcode.revamped.utils.GamepadEx;

public class TeleOp extends OpModeCommand {
    @Override
    public void initialize() {
        Robot robot = new Robot(hardwareMap);
        TeleOpStateHandler<RobotStateHandler.CycleState> tsh = RobotStateHandler.createTeleOpStateHandler(robot);
        GamepadEx gamepad_1 = new GamepadEx(gamepad1);
        GamepadEx gamepad_2 = new GamepadEx(gamepad2);

        ButtonMapper mapper = new ButtonMapper()
                .put(gamepad_1.a.risingEdge(), new Instant(() -> TrackingThread.trackTurret = true))
                .put(gamepad_1.y.risingEdge(), new Instant(() -> tsh.setForce(!tsh.isForce())))
                .put(gamepad_1.dpad_up.risingEdge(), tsh.setting(robot::shootFar))
                .put(gamepad_1.dpad_down.risingEdge(), tsh.setting(robot::shootNear))
                .put(gamepad_1.dpad_left.risingEdge(), tsh.setting(robot::shootMedium))
                .put(gamepad_1.right_trigger.greaterThan(0.3f).risingEdge(), tsh.task(
                        () -> robot.turret.setTargetPosition(
                                        robot.turret.getTargetPosition() + (int) (20 * gamepad1.right_trigger)
                                ), new int[] {1, 1, 0}
                        )
                )
                .put(gamepad_1.left_trigger.greaterThan(0.3f).risingEdge(), tsh.task(
                        () -> robot.turret.setTargetPosition(
                                        robot.turret.getTargetPosition() - (int) (20 * gamepad1.left_trigger)
                                ), new int[] {1, 1, 0}
                        )
                )
                .put(gamepad_1.right_bumper.risingEdge(), tsh.task(robot.turret::next, new int[] {1, 1, 0}))
                .put(gamepad_1.left_bumper.risingEdge(), tsh.task(robot.turret::previous, new int[] {1, 1, 0}))
                .put(gamepad_1.b.risingEdge(), new Sequential(
                                new WaitUntil(robot.turret.limitSwitch::state),
                                new Instant(robot.turret::resetPosition)
                        )
                )
                .put(gamepad_1.x.risingEdge(), new Instant(() -> {tsh.abortTransition(); robot.popper.neutral();}))
                .put(gamepad_2.b.risingEdge().and(() -> tsh.atState(RobotStateHandler.CycleState.DRIVE_TO_SHOOT)
                        || !robot.intakeMotor.atPower(IntakeMotor.INTAKE)), tsh.runTransition(() -> {
                            robot.flywheel.medium();
                            robot.intakeMotor.intake();
                            robot.table.reset();
                        },
                        RobotStateHandler.CycleState.INTAKE)
                )
                .put(gamepad_2.y.risingEdge(), robot.sort())
                .put(gamepad_2.dpad_right.risingEdge(), new Instant(robot.intakeMotor::outtake))
                .put(gamepad_2.right_trigger.greaterThan(0.3f).risingEdge(), new Instant(robot.intakeMotor::stop))
                .put(gamepad_2.right_bumper.risingEdge(), tsh.task(() -> {
                    robot.setRobotState(RobotStateHandler.IntakeMessage.SORTING);
                    robot.table.next();
                }, new int[] {1,0,0}))
                .put(gamepad_2.left_bumper.risingEdge(), tsh.task(() -> {
                    robot.setRobotState(RobotStateHandler.IntakeMessage.SORTING);
                    robot.table.previous();
                }, new int[] {1,0,0}))
                .put(gamepad_2.x.risingEdge().and(() -> tsh.atState(RobotStateHandler.CycleState.INTAKE)
                        && robot.popper.atPos(Popper.NEUTRAL)), tsh.runTransition(
                            new Sequential(
                                    new Instant(() -> {
                                        robot.tableCompartments.intakeThread.updateColors();
                                        robot.intakeMotor.stop();
                                        robot.popper.pop();
                                    }),
                                    new Wait(250)
                            ),
                        RobotStateHandler.CycleState.DRIVE_TO_SHOOT)
                )
                .put(gamepad_2.dpad_up.risingEdge(),
                        new Sequential(
                                tsh.runTransition(() -> {}, RobotStateHandler.CycleState.SHOOT),
                                tsh.runTransition(
                                        new Sequential(
                                                robot.shootAll(() -> {
                                                    if (robot.hood.atPos(Hood.FAR_PRESET)) return 100.0;
                                                    return 0.0;
                                                }),
                                                robot.resetAfterShooting()
                                        ), RobotStateHandler.CycleState.INTAKE
                                )
                        )
                )
                .put(gamepad_2.dpad_down.risingEdge(),
                        new Sequential(
                                tsh.runTransition(() -> {}, RobotStateHandler.CycleState.SHOOT),
                                tsh.runTransition(
                                        new Sequential(
                                                robot.shootAll(250),
                                                robot.resetAfterShooting()
                                        ), RobotStateHandler.CycleState.INTAKE
                                )
                        )
                );

        schedule(
                new Infinite(() -> {
                    robot.update();
                    mapper.update();
                }),
                new Sequential(
                        new WaitUntil(() -> !opModeInInit()),
                        new Instant(robot::init)
                )
        );
    }

    @Override
    public void end() {
        Turret.startPos = 0;
    }
}
