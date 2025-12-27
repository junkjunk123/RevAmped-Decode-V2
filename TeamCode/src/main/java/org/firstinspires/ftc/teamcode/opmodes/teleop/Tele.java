package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.TeleOpStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Popper;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.TrackingThread;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.utils.commands.ButtonMapper;
import org.firstinspires.ftc.teamcode.utils.GamepadEx;

@Config
@TeleOp(name = "DCTeleOp")
public class Tele extends OpModeCommand {
    @Override
    public void initialize() {
        Robot robot = new Robot(hardwareMap);
        TeleOpStateHandler tsh = RobotStateHandler.createTeleOpStateHandler(robot);
        GamepadEx gamepad_1 = new GamepadEx(gamepad1);
        GamepadEx gamepad_2 = new GamepadEx(gamepad2);

        ButtonMapper mapper = new ButtonMapper()
                .put(gamepad_1.a.risingEdge(), new Instant(() -> {
                        TrackingThread.trackTurret = true;}))
                .put(gamepad_1.y.risingEdge(), new Instant(() -> tsh.setForce(!tsh.isForce())))
                .put(gamepad_1.dpad_up.risingEdge(), new Instant(robot::shootFar))
                .put(gamepad_1.dpad_down.risingEdge(), new Instant(robot::shootNear))
                .put(gamepad_1.dpad_left.risingEdge(), new Instant(robot::shootMedium))
                .put(gamepad_1.right_trigger.greaterThan(0.3f).risingEdge(),
                        new Instant(
                                () -> robot.turret.runToPos(
                                        robot.turret.getTargetPosition() + (int) (20 * gamepad1.right_trigger)
                                )
                        )
                )
                .put(gamepad_1.left_trigger.greaterThan(0.3f).risingEdge(), new Instant(
                                () -> robot.turret.runToPos(
                                        robot.turret.getTargetPosition() - (int) (20 * gamepad1.left_trigger)
                                )
                        )
                )
                .put(gamepad_1.right_bumper.risingEdge(), new Instant(robot.turret::next))
                .put(gamepad_1.left_bumper.risingEdge(), new Instant(robot.turret::previous))
                .put(gamepad_1.b.risingEdge(), new Sequential(
                                new WaitUntil(robot.turret.limitSwitch::state),
                                new Instant(robot.turret::resetPosition)
                        )
                )
                .put(gamepad_1.x.risingEdge(), new Instant(() -> {
                    tsh.abortTransition();
                    robot.popper.neutral();
                    tsh.setCurrentState(RobotStateHandler.IntakeMessage.SORTING);
                }))
                .put(gamepad_2.b.risingEdge().and(() -> tsh.atState(RobotStateHandler.CycleState.DRIVE_TO_SHOOT)
                                || !robot.intakeMotor.atPower(IntakeMotor.INTAKE)), new Sequential(
                                new Instant(() -> {
                                    robot.flywheel.medium();
                                    robot.intakeMotor.intake();
                                }),
                                robot.table.reset())
                )
                .put(gamepad_2.y.risingEdge(), robot.sort())
                .put(gamepad_2.dpad_right.risingEdge(), new Instant(robot.intakeMotor::outtake))
                .put(gamepad_2.right_trigger.greaterThan(0.3f).risingEdge(), new Instant(robot.intakeMotor::stop))
                .put(gamepad_2.right_bumper.risingEdge(), new Sequential(
                    new Instant(() -> robot.setRobotState(RobotStateHandler.IntakeMessage.SORTING)),
                    robot.table.next()
                ))
                .put(gamepad_2.left_bumper.risingEdge(), new Sequential(
                        new Instant(() -> robot.setRobotState(RobotStateHandler.IntakeMessage.SORTING)),
                        robot.table.previous()
                ))
                .put(gamepad_2.x.risingEdge().and(() -> tsh.atState(RobotStateHandler.CycleState.INTAKE)
                        && robot.popper.atState(Popper.PopperState.NEUTRAL)), new Sequential(
                                new Instant(() -> {
                                    robot.tableCompartments.intakeThread.updateColors();
                                    robot.intakeMotor.stop();
                                    robot.popper.pop();
                                }),
                                new Wait(250)
                        )
                )
                .put(gamepad_2.dpad_up.risingEdge(), new Sequential(
                                robot.shootAll(() -> {
                                    if (robot.hood.atPos(Hood.FAR_PRESET))
                                        return 100.0;
                                    return 0.0;
                                }),
                                robot.resetAfterShooting()
                        )
                )
                .put(gamepad_2.dpad_down.risingEdge(), new Sequential(
                                robot.shootAll(175),
                                robot.resetAfterShooting()
                        )
                );

        schedule(
                new Infinite(() -> {
                    robot.update();
                    mapper.update();
                    robot.drivetrain.arcadeDrive(gamepad1);
                }),
                new Sequential(
                        new WaitUntil(() -> !opModeInInit()),
                        robot.init()
                )
        );
    }

    @Override
    public void execute() {
        telemetry.addData("turret", Robot.INSTANCE.turret.getTargetPosition());
        telemetry.addData("popper", Robot.INSTANCE.popper.atPos(Popper.NEUTRAL));
        telemetry.update();
    }

    @Override
    public void end() {
        Turret.startPos = 0;
    }
}
