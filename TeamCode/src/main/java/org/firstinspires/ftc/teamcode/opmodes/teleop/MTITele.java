package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.TeleOpStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeDistanceSensors;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.TrackingThread;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
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
    public static int rumbleMS;
    public static boolean calibrateTurret;
    public static boolean outreach;

    public static double turretPos;
    public static boolean disableThresholdTrackChange;
    private TrackingThread autoTrack;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);
        gamepad_1 = new GamepadEx(gamepad1);
        gamepad_2 = new GamepadEx(gamepad2);
        tsh = RobotStateHandler.createTeleOpStateHandler(robot);
        //set gamepad2 color
        if (IntakeDistanceSensors.useSensors){
            gamepad2.setLedColor(1,0,0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad2.setLedColor(0.988,0.039,0.706, Gamepad.LED_DURATION_CONTINUOUS);
        }
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
        //updates
        gamepad_1.update();
        gamepad_2.update();
        autoTrack.update();

        //Auto Transfer (Robot is at intake AND the sensors are on AND robot has three)
        if ((robot.intake.distanceSensors.isOn()  && robot.intake.hasThree() && tsh.atState(RobotStateHandler.CycleState.INTAKE))){
            schedule(
                    tsh.runTransition(
                            () -> {
                                schedule(
                                    new Parallel(
                                        robot.transfer(),
                                        new Instant(() -> gamepad_1.rumble(rumbleMS))
                                    )
                                );
                            },
                            RobotStateHandler.CycleState.DRIVE_TO_SHOOT
                    )
            );
        }

        //Stop transfer motor (Robot is at intake AND the sensors are on AND robot has a ball in the transfer)
        if (robot.intake.distanceSensors.isOn() && robot.intake.ballInTransfer() && tsh.atState(RobotStateHandler.CycleState.INTAKE)){
            schedule(new Instant(robot::stopFeeder));
        }

        //====================GAMEPAD_1===================
        //Gate open
        if (gamepad_1.left_bumper.isRisingEdge()){
            schedule(
                    tsh.runTransition(
                        robot.gate.open(),
                        RobotStateHandler.CycleState.DRIVE_TO_SHOOT
                    )
            );
        }
        //Hold Shoot
        if (gamepad_1.right_bumper.isRisingEdge()){
            schedule(
                    tsh.runTransition(
                            new Conditional(() -> Robot.shootingFar,
                                    new Parallel(
                                        robot.farHoodComp(),
                                        new Instant(robot::transferShootFar)
                                    ),
                                    new Parallel(
                                        robot.hoodComp(),
                                        new Instant(robot::transferShoot)
                                    )
                            ),
                            RobotStateHandler.CycleState.SHOOT
                    )
            );
        }
        //Resolve after hold is done
        if (gamepad_1.right_bumper.isFallingEdge()){
            schedule(
                    tsh.runTransition(
                            new Sequential(
                                    robot.stopCleanup(),
                                    new Conditional(() -> IntakeDistanceSensors.useSensors,
                                            new Instant(robot::intake),
                                            new Instant(() -> robot.intake(true))
                                    ),
                                    new Instant(() -> {
                                        disableThresholdTrackChange = false;
                                        TrackingThread.trackTurret = true;
                                        TrackingThread.trackHood = true;
                                    })
                            ),
                            RobotStateHandler.CycleState.INTAKE
                    )
            );
        }
        //Reset Point
        if (gamepad_1.back.isRisingEdge()){
            robot.drivetrain.follower.setPose(Drivetrain.resetPose);
        }
        //Far Preset
        if (gamepad_1.dpad_up.isRisingEdge()){
            disableThresholdTrackChange = true;
            TrackingThread.trackHood = false;
            robot.shootFar();
        }
        //Near Preset
        if (gamepad_1.dpad_down.isRisingEdge()){
            disableThresholdTrackChange = true;
            TrackingThread.trackHood = false;
            robot.shootNear();
        }
        //Medium Preset
        if (gamepad_1.dpad_left.isRisingEdge()){
            disableThresholdTrackChange = true;
            TrackingThread.trackHood = false;
            robot.shootMedium();
        }
        //Corner Preset
        if (gamepad_1.dpad_right.isRisingEdge()){
            disableThresholdTrackChange = true;
            TrackingThread.trackHood = false;
            robot.shootCorner();
        }
        //Next Turret Preset
        if (gamepad_1.right_trigger_button.isRisingEdge()){
            TrackingThread.trackTurret = false;
            robot.turret.next();
        }
        //Previous Turret Preset
        if (gamepad_1.left_trigger_button.isRisingEdge()){
            TrackingThread.trackTurret = false;
            robot.turret.previous();
        }


        //====================GAMEPAD_2===================
        //Intake
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
        //Outtake
        if (gamepad_2.right_bumper.isRisingEdge()){
            schedule(new Instant(robot::outtake));
        }
        //Reverse transfer (not used)
        if (gamepad_2.a.isRisingEdge()){
            schedule(robot.reverseTransfer());
        }
        //toggles auto transfer
        if (gamepad_2.x.isRisingEdge()){
            IntakeDistanceSensors.useSensors = !IntakeDistanceSensors.useSensors;

            if (IntakeDistanceSensors.useSensors){
                gamepad2.setLedColor(1,0,0, Gamepad.LED_DURATION_CONTINUOUS);
            } else {
                gamepad2.setLedColor(0.988,0.039,0.706, Gamepad.LED_DURATION_CONTINUOUS);
            }

            //setting the new power of the feeder if it is on so we don't kill the transfer wheels
            if (!IntakeDistanceSensors.useSensors && robot.feederWheel.getPower() != 0){
                robot.feederWheel.intakeSlow();
            } else if (IntakeDistanceSensors.useSensors && robot.feederWheel.getPower() != 0){
                robot.feederWheel.intake();
            }

            gamepad_2.rumble(rumbleMS);
        }

        //====================MISC===================
        //Confirm turret calibration
        if (gamepad_2.dpad_down.isRisingEdge() && calibrateTurret){
            schedule(new Instant(() -> {
                    robot.turret.setPosition(turretPos);
                    TrackingThread.trackTurret = false;
            }));
        }
        //Stop tele for outreach
        if (gamepad_2.back.isRisingEdge() && outreach){
            requestOpModeStop();
        }

        //Telemetry
        telemetry.addData("sensors", Arrays.toString(robot.intake.getStates()));
        telemetry.addData("error",robot.flywheel.getError());
        telemetry.addData("velocity",robot.flywheel.getVelocity());
        telemetry.addData("tsh",tsh.currentState().toString());
        telemetry.addData("shootingFar",Robot.shootingFar);
        telemetry.addData("trackHood",TrackingThread.trackHood);
        telemetry.addData("trackTurret",TrackingThread.trackTurret);
        telemetry.addData("disableThresholdTrack",disableThresholdTrackChange);
    }
}