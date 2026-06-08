package org.firstinspires.ftc.teamcode.opmodes.teleop;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.TeleOpStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeDistanceSensors;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.TrackingThread;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.pedro.ColoredDecodePose;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.commands.GamepadEx;
import org.firstinspires.ftc.teamcode.utils.data.FloatSupplier;

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
    private TrackingThread autoTrack;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);
        gamepad_1 = new GamepadEx(gamepad1);
        gamepad_2 = new GamepadEx(gamepad2);
        tsh = RobotStateHandler.createTeleOpStateHandler(robot);
        //to default to manaul turret
//        RobotStateHandler.CycleState.DriveToShoot.toggleDefault();
        autoTrack = new TrackingThread(robot);
        gamepad_1.left_trigger_button(FloatSupplier::isPress);
        gamepad_1.right_trigger_button(FloatSupplier::isPress);
        gamepad_2.left_trigger_button(FloatSupplier::isPress);
        gamepad_2.right_trigger_button(FloatSupplier::isPress);

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
        autoTrack.update();

        if (gamepad_2.b.isRisingEdge()){
            schedule(tsh.runTransition(
                    new Conditional(
                            () -> IntakeDistanceSensors.useSensors,
                            new Instant(robot::intake),
                            new Instant(() -> robot.intake(true))
                    ),
                    RobotStateHandler.CycleState.INTAKE)

            );
        }

        if (gamepad_2.right_bumper.isRisingEdge()){
            schedule(new Instant(robot::outtake));
        }

        if (gamepad_2.x.isRisingEdge() || (robot.intake.distanceSensors.isOn()  && robot.intake.hasThree() && tsh.atState(RobotStateHandler.CycleState.INTAKE))){
            schedule(
                tsh.runTransition(
                    robot.transfer(),
                    RobotStateHandler.CycleState.DRIVE_TO_SHOOT
                )
            );
        }

        if (gamepad_1.left_bumper.isRisingEdge()){
            schedule(
                tsh.runTransition(
                    robot.gate.open(),
                    RobotStateHandler.CycleState.DRIVE_TO_SHOOT
                )
            );
        }

        if (gamepad_1.right_bumper.isRisingEdge()){
            schedule(
                tsh.runTransition(
                    new Conditional(() -> robot.shootingFar,
                        new Instant(robot::transferShootFar),
                        new Instant(robot::transferShoot)),
                    RobotStateHandler.CycleState.SHOOT
                )
            );
        }

        if (gamepad_1.right_bumper.isFallingEdge()){
            schedule(
                tsh.runTransition(
                    new Sequential(
                        robot.stopCleanup(),
                        new Instant(() -> {
                            robot.intake();
                            TrackingThread.trackTurret = true;
                            TrackingThread.trackHood = true;
                        })
                    ),
                    RobotStateHandler.CycleState.INTAKE
                )
            );
        }

        if (gamepad_2.y.isRisingEdge() || (robot.intake.distanceSensors.isOn() && robot.intake.ballInTransfer() && tsh.atState(RobotStateHandler.CycleState.INTAKE))){
            schedule(new Instant(robot::stopFeeder));
        }

        if (gamepad_2.a.isRisingEdge()){
            schedule(robot.reverseTransfer());
        }

        if (gamepad_2.dpad_down.isRisingEdge() && calibrateTurret){
            schedule(new Instant(() -> {
                    robot.turret.setPosition(turretPos);
                    TrackingThread.trackTurret = false;
            }));
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
                                new Instant(() -> {
                                    TrackingThread.trackHood = true;
                                    TrackingThread.trackTurret = true;
                                }),
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

        if (gamepad_1.back.isRisingEdge()){
            robot.drivetrain.follower.setPose(Drivetrain.resetPose);
        }

        if (gamepad_1.dpad_up.isRisingEdge()){
            TrackingThread.trackHood = false;
            robot.shootFar();
        }

        if (gamepad_1.dpad_down.isRisingEdge()){
            TrackingThread.trackHood = false;
            robot.shootNear();
        }

        if (gamepad_1.dpad_left.isRisingEdge()){
            TrackingThread.trackHood = false;
            robot.shootMedium();
        }

        if (gamepad_1.dpad_right.isRisingEdge()){
            TrackingThread.trackHood = false;
            robot.shootCorner();
        }

        if (gamepad_1.right_trigger_button.isRisingEdge()){
            TrackingThread.trackTurret = false;
            robot.turret.next();
        }

        if (gamepad_1.left_trigger_button.isRisingEdge()){
            TrackingThread.trackTurret = false;
            robot.turret.previous();
        }

        telemetry.addData("sensors", Arrays.toString(robot.intake.getStates()));
        telemetry.addData("error",robot.flywheel.getError());
        telemetry.addData("velocity",robot.flywheel.getVelocity());
        telemetry.addData("tsh",tsh.currentState().toString());
        telemetry.addData("shootingFar",robot.shootingFar);
        telemetry.addData("trackHood",TrackingThread.trackHood);
        telemetry.addData("trackTurret",TrackingThread.trackTurret);
        telemetry.addData("Y",robot.drivetrain.follower.getPose().getY());
    }
}