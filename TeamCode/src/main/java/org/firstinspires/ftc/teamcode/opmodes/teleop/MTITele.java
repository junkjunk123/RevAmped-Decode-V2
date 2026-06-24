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
import org.firstinspires.ftc.teamcode.mechanisms.shooter.FlywheelController;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.TrackingThread;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.commands.GamepadEx;
import org.firstinspires.ftc.teamcode.utils.data.FloatSupplier;
import org.firstinspires.ftc.teamcode.utils.math.projectile.SimpleShooterMath;

@Config
@TeleOp(name = "MTI-TeleOp")
public class MTITele extends OpModeCommand {
    private GamepadEx gamepad_1;
    private GamepadEx gamepad_2;
    private Robot robot;
    private TeleOpStateHandler tsh;
    public static int rumbleMS;
    public static boolean calibration;
    public static boolean outreach;

    public static double turretPos;
    public static boolean disableThresholdTrackChange;
    public static float DRIVER_TURRET_OFFSET;
    public static boolean canShoot; //for if the robot is too close to the goal
    private TrackingThread autoTrack;
    private boolean record = false;
    private double maxVel = 0;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);
        gamepad_1 = new GamepadEx(gamepad1);
        gamepad_2 = new GamepadEx(gamepad2);
        tsh = RobotStateHandler.createTeleOpStateHandler(robot);
        updateGP2Color();
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
                new Conditional(() -> IntakeDistanceSensors.useSensors,
                        new Instant(robot::intake),
                        new Instant(() -> robot.intake(true))
                ),
                new Instant(robot.flywheel::near),
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
        if (gamepad_1.left_bumper.isRisingEdge() || gamepad_2.x.isRisingEdge() || (robot.intake.distanceSensors.isOn()  && robot.intake.hasThree() && tsh.atState(RobotStateHandler.CycleState.INTAKE))){
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

        //resting threshold
        if (!disableThresholdTrackChange) {
            if (Robot.shootingFar && tsh.atState(RobotStateHandler.CycleState.INTAKE) && TrackingThread.trackHood) {
                TrackingThread.trackHood = false;
                robot.flywheel.medium();
            }

            if ((!Robot.shootingFar || (!tsh.atState(RobotStateHandler.CycleState.INTAKE)) && !TrackingThread.trackHood)) {
                TrackingThread.trackHood = true;
            }
        }

        //Stop transfer motor (Robot is at intake AND the sensors are on AND robot has a ball in the transfer)
        if (robot.intake.distanceSensors.isOn() && robot.intake.ballInTransfer() && tsh.atState(RobotStateHandler.CycleState.INTAKE)){
            schedule(new Instant(robot::stopFeeder));
        }

        //====================GAMEPAD_1===================
        //Gate open (now in auto transfer block)
//        if (gamepad_1.left_bumper.isRisingEdge()){
//            schedule(
//                    tsh.runTransition(
//                        robot.gate.open(),
//                        RobotStateHandler.CycleState.DRIVE_TO_SHOOT
//                    )
//            );
//        }

        //Hold Shoot
        if (gamepad_1.right_bumper.isRisingEdge()){
            if (canShoot || calibration) {
                schedule(
                    new Sequential(
                        new WaitUntil(() -> robot.flywheel.getController().getMode().equals(FlywheelController.Mode.HOLD)),
                        tsh.runTransition(
                            new Conditional(() -> Robot.shootingFar,
                                robot.autoShootFar(),
                                robot.autoShoot()
                            ),
                            RobotStateHandler.CycleState.SHOOT
                        )
                    )
                );
            } else{
                gamepad_1.rumble(rumbleMS);
            }
        }

        //Resolve after hold is done rumbles if too close to goal
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
                            SimpleShooterMath.turretFarOffset = 0;
                        })
                    ),
                    RobotStateHandler.CycleState.INTAKE
                )
            );
        }
        //Reset Point
        if (gamepad_1.back.isRisingEdge()){
            robot.drivetrain.follower.setPose(Drivetrain.resetPose.getPose());
        }
        //Far Preset
        if (gamepad_1.dpad_up.isRisingEdge()){
            if (Globals.allianceColor.equals(AllianceColor.Red)){
                SimpleShooterMath.turretFarOffset = 3/255f;
            } else {
                SimpleShooterMath.turretFarOffset = -2/255f;
            }
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
        //Toggle sotm
        if (gamepad_1.right_trigger_button.isRisingEdge()) {
            TrackingThread.velocityCompensation = !TrackingThread.velocityCompensation;
        }

        if (gamepad1.left_stick_y > 0.3f && !TrackingThread.velocityCompensation){
            Robot.sotmTurretComp = true;
        } else{
            Robot.sotmTurretComp = false;
        }

        //====================GAMEPAD_2===================
        //Intake
        if (gamepad_2.b.isRisingEdge()){
            schedule(tsh.runTransition(
                    new Sequential(
                        robot.gate.close(),
                        new Conditional(
                                () -> IntakeDistanceSensors.useSensors,
                                new Instant(robot::intake),
                                new Instant(() -> robot.intake(true))
                        )
                    ),
                    RobotStateHandler.CycleState.INTAKE)

            );
        }
        //Outtake
        if (gamepad_2.right_trigger_button.isRisingEdge()){
            schedule(new Instant(robot::outtake));
        }
        //toggles auto transfer
        if (gamepad_2.left_bumper.isRisingEdge()){
            IntakeDistanceSensors.useSensors = !IntakeDistanceSensors.useSensors;

            //setting the new power of the feeder if it is on so we don't kill the transfer wheels
            if (!IntakeDistanceSensors.useSensors && robot.feederWheel.getPower() != 0){
                robot.feederWheel.intakeSlow();
            } else if (IntakeDistanceSensors.useSensors && robot.feederWheel.getPower() != 0){
                robot.feederWheel.intake();
            }

            updateGP2Color();
            gamepad_2.rumble(rumbleMS);
        }
        //toggle stick sotm
        if (gamepad_2.right_bumper.isRisingEdge()){
            Robot.enableDriverSOTM = !Robot.enableDriverSOTM;

            updateGP2Color();
            gamepad_2.rumble(rumbleMS);
        }
        //reset driver offsets
        if (gamepad_2.left_trigger_button.isRisingEdge()){
            Robot.hoodFineTune = 0;
            Robot.flywheelFineTune = 0;
            gamepad_2.rumble(rumbleMS);
        }
        //flywheel driver offsets
        if (gamepad_2.dpad_up.isRisingEdge()){
            Robot.flywheelFineTune+=15;
        } else if (gamepad_2.dpad_down.isRisingEdge()){
            Robot.flywheelFineTune-=15;
        }
        //hood driver offsets
        if (gamepad_2.dpad_right.isRisingEdge()){
            Robot.hoodFineTune-=3/255f;
        } else if (gamepad_2.dpad_left.isRisingEdge()){
            Robot.hoodFineTune+=3/255f;
        }
        //====================MISC===================
        //Confirm turret calibration
        if (gamepad_2.dpad_down.isRisingEdge() && calibration){
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
//        telemetry.addData("flywheel error",robot.flywheel.getError());
//        telemetry.addData("flywheel velocity",robot.flywheel.getFilteredVelocity());
//        telemetry.addData("flywheel target",robot.flywheel.getTargetVelocity());
//        telemetry.addData("hood pos",robot.hood.getPosition());
//        telemetry.addData("intake power",robot.intake.intakeMotor.getPower());
//        telemetry.addData("feeder velocity",robot.feederWheel.getVelocity());
//        telemetry.addData("feeder target",robot.feederWheel.getTargetVelocity());
//        telemetry.addData("feeder error",robot.feederWheel.getTargetVelocity()-robot.feederWheel.getVelocity());

        //distance sensor testing
//        telemetry.addData("states",Arrays.toString(robot.intake.getStates()));
//        telemetry.addData("sensor",robot.intake.distanceSensors.getCurrentSensor());
//        telemetry.addData("three?",robot.intake.hasThree());
//        telemetry.addData("two?",robot.intake.hasTwo());
//        telemetry.addData("sensor states",Arrays.toString(robot.intake.distanceSensors.getSensorStates()));
    }

    public void updateGP2Color(){
        int green = IntakeDistanceSensors.useSensors ? 1 : 0;
        int red = Robot.enableDriverSOTM ? 1 : 0;
        gamepad2.setLedColor(red,green,0,Gamepad.LED_DURATION_CONTINUOUS);
    }
}