package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.mechanisms.vision.DecodeLimelight;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.math.calc.Vector2D;

@Disabled
@TeleOp
public class LimelightTest extends OpMode {
    private DecodeLimelight limelight;
    private Robot robot;

    @Override
    public void init() {
        Globals.init(telemetry);
        robot = new Robot(hardwareMap);
        limelight = new DecodeLimelight(hardwareMap);
        limelight.setCurrentPipeline(DecodeLimelight.Pipeline.SHOOTING_ALIGNMENT);
        robot.turret.setPosition(ServoTurret.REST);
    }

    @Override
    public void loop() {
        robot.update();
        robot.drivetrain.arcadeDrive(gamepad1);
        limelight.update();

        if (limelight.getCurrentPipeline() != DecodeLimelight.Pipeline.SHOOTING_ALIGNMENT)
            limelight.setCurrentPipeline(DecodeLimelight.Pipeline.SHOOTING_ALIGNMENT);

        //RobotStateHandler.CycleState.DRIVE_TO_SHOOT.update();

        Vector offset = limelight.getOffsets();
        Localizer pinpoint = robot.drivetrain.follower.poseTracker.getLocalizer();
        pinpoint.update();
        Vector targ = DecodeLimelight.APRILTAG_POSE.getPose().getAsVector();
        double bearing = offset.getTheta();
        double robotHeading = pinpoint.getPose().getHeading();
        double turretRad = ServoTurret.ticksToRad(robot.turret.getPosition());
        double viewRad = ServoTurret.ticksToRad(robot.turret.getPosition()) + robotHeading + Math.PI;
        telemetry.addData("turretTicks", robot.turret.getPosition());
        telemetry.addData("turretRad", turretRad);
        double alpha = viewRad - bearing;
        Vector tagOffset = new Vector(offset.getXComponent(), alpha);
        Vector camPos = targ.minus(tagOffset);
        Vector camOffset = new Vector(DecodeLimelight.CENTER_OFFSET, viewRad);
        Vector turretPos = camPos.minus(camOffset);
        Vector turretOffset;
        if (Turret.TURRET_OFFSET >= 0) turretOffset = new Vector2D(Turret.TURRET_OFFSET, robotHeading);
        else turretOffset = new Vector2D(Turret.TURRET_OFFSET, Math.PI + robotHeading);
        Vector robotPos = turretPos.minus(turretOffset);

        telemetry.addData("offsets", offset);
        telemetry.addData("pinpoint pos", pinpoint.getPose());
        telemetry.addData("view rad", viewRad);
        telemetry.addData("alpha", alpha);
        telemetry.addData("camPos", camPos);
        telemetry.addData("turretPos", turretPos);
        telemetry.addData("robotPos", robotPos);
        telemetry.update();
    }
}
