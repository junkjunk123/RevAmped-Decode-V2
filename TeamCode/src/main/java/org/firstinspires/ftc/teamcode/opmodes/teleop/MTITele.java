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
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.TeleOpStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeDistanceSensors;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.commands.GamepadEx;

import java.util.Arrays;

@Config
@TeleOp(name = "MTI-TeleOp")
public class MTITele extends OpModeCommand {
    private GamepadEx gamepad_1;
    private GamepadEx gamepad_2;
    private Robot robot;
    private TeleOpStateHandler tsh;
    public static boolean calibrateTurret;

    public static double turretPos;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        Globals.init(telemetry);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);
        gamepad_1 = new GamepadEx(gamepad1);
        gamepad_2 = new GamepadEx(gamepad2);
        tsh = RobotStateHandler.createTeleOpStateHandler(robot);
        RobotStateHandler.CycleState.DriveToShoot.toggleDefault();
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
            schedule(tsh.runTransition(
                    new Conditional(
                            () -> IntakeDistanceSensors.useSensors,
                            new Instant(robot::intake),
                            new Instant(() -> robot.intake(true))
                    )
                    ,RobotStateHandler.CycleState.INTAKE)

            );
        }

        if (gamepad_2.right_bumper.isRisingEdge()){
            schedule(new Instant(robot::outtake));
        }

        if (gamepad_2.x.isRisingEdge() || (robot.intake.distanceSensors.isOn()  && robot.intake.hasThree() && tsh.atState(RobotStateHandler.CycleState.INTAKE))){
            schedule(
                tsh.runTransition(
                    robot.transfer()
                    , RobotStateHandler.CycleState.DRIVE_TO_SHOOT)
            );
        }

        if (gamepad_2.y.isRisingEdge() || (robot.intake.distanceSensors.isOn() && robot.intake.ballInTransfer() && tsh.atState(RobotStateHandler.CycleState.INTAKE))){
            schedule(new Instant(robot::stopFeeder));
        }

        if (gamepad_2.a.isRisingEdge()){
            schedule(robot.reverseTransfer());
        }

        if (gamepad_2.dpad_up.isRisingEdge() && calibrateTurret){
            schedule(new Instant(() -> robot.turret.setPosition(turretPos)));
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

        if (gamepad_2.dpad_up.isRisingEdge() && tsh.atState(RobotStateHandler.CycleState.DRIVE_TO_SHOOT)){
            schedule(
                    tsh.runTransition(
                        new Sequential(
                                new Conditional(
                                    () -> robot.shootingFar,
                                    robot.autoShootFar(),
                                    robot.autoShoot()
                                ),
                                robot.resetAfterShooting(),
                                //change this after we use sensors
                                new Conditional(
                                    () -> IntakeDistanceSensors.useSensors,
                                    new Instant(() -> robot.intake()),
                                    new Instant(() -> robot.intake(true))
                                )
                        )
                    , RobotStateHandler.CycleState.INTAKE)
            );
        }

        if (gamepad_1.dpad_up.isRisingEdge()){
            robot.shootFar();
        }

        if (gamepad_1.dpad_down.isRisingEdge()){
            robot.shootNear();
        }

        if (gamepad_1.dpad_left.isRisingEdge()){
            robot.shootMedium();
        }

        if (gamepad_1.dpad_right.isRisingEdge()){
            robot.shootCorner();
        }

        if (gamepad_1.right_bumper.isRisingEdge()){
            robot.turret.next();
        }

        if (gamepad_1.left_bumper.isRisingEdge()){
            robot.turret.previous();
        }

        telemetry.addData("sensors", Arrays.toString(robot.intake.getStates()));
        telemetry.addData("on",robot.intake.distanceSensors.isOn());
        telemetry.addData("error",robot.flywheel.getError());
        telemetry.addData("velocity",robot.flywheel.getVelocity());
        telemetry.addData("isShootingFar",robot.shootingFar);
        telemetry.addData("tsh",tsh.currentState().toString());
    }
}