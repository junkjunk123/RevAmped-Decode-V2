package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeDistanceSensors;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.commands.GamepadEx;

import java.util.Arrays;

@Config
@TeleOp
public class HardwareTele extends OpModeCommand {
    private GamepadEx gamepad_1;
    private GamepadEx gamepad_2;
    private Robot robot;
    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        Globals.init(telemetry);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);
        gamepad_1 = new GamepadEx(gamepad1);
        gamepad_2 = new GamepadEx(gamepad2);

        schedule(new Infinite(() -> {
            robot.update();
            robot.drivetrain.arcadeDrive(gamepad1);
            telemetry.update();
        }));

        schedule(
            new Sequential(
                new WaitUntil(() -> !opModeInInit()),
                new Instant(robot::initialize),
                new Wait(500)
            )
        );
    }

    @Override
    public void execute(){
        gamepad_1.update();
        gamepad_2.update();

        if (gamepad_2.b.isRisingEdge()){
            schedule(
                    new Conditional(
                            () -> IntakeDistanceSensors.useSensors,
                            new Instant(robot::intake),
                            new Instant(() -> robot.intake(true))
                    )

            );
        }

        if (gamepad_2.right_bumper.isRisingEdge()){
            schedule(new Instant(robot::outtake));
        }

        if (gamepad_2.x.isRisingEdge() || (robot.intake.distanceSensors.isOn() && !robot.intake.distanceSensors.shouldPause() && robot.intake.hasThree())){
            schedule(robot.transfer());
            robot.intake.distanceSensors.stop();
        }

        if (gamepad_2.y.isRisingEdge() || (robot.intake.distanceSensors.isOn() && robot.intake.ballInTransfer())){
            schedule(new Instant(robot::stopFeeder));
        }

        if (robot.intake.distanceSensors.shouldPause()) {
            schedule(
                    new Sequential(
                            new Instant(() -> robot.intake.distanceSensors.stop()),
                            new Wait(300),
                            new Instant(() -> {
                                robot.intake.distanceSensors.start();
                                robot.intake.distanceSensors.update();
                            })
                    )
            );
        }

        if (gamepad_2.dpad_up.isRisingEdge()){
            schedule(
                    new Sequential(
                            robot.autoShoot(),
                            robot.resetAfterShooting(),
                            //change this after we use sensors
                            new Conditional(
                                () -> IntakeDistanceSensors.useSensors,
                                new Instant(() -> robot.intake()),
                                new Instant(() -> robot.intake(true))
                            )
                    )
            );
        }

        if (gamepad_1.dpad_up.isRisingEdge()){
            robot.flywheel.far();
            robot.hood.far();
        }

        if (gamepad_1.dpad_down.isRisingEdge()){
            robot.flywheel.near();
            robot.hood.near();
        }

        if (gamepad_1.dpad_left.isRisingEdge()){
            robot.flywheel.medium();
            robot.hood.medium();
        }

        if (gamepad_1.dpad_right.isRisingEdge()){
            robot.flywheel.corner();
            robot.hood.corner();
        }

        if (gamepad_1.right_bumper.isRisingEdge()){
            robot.turret.next();
        }

        if (gamepad_1.left_bumper.isRisingEdge()){
            robot.turret.previous();
        }

        telemetry.addData("sensors", Arrays.toString(robot.intake.getStates()));
        telemetry.addData("on",robot.intake.distanceSensors.isOn());
    }
}