package org.firstinspires.ftc.teamcode.revamped.opmodes.auto;

import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;

import org.firstinspires.ftc.teamcode.revamped.Robot;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.DecodeLimelight;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.revamped.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.revamped.opmodes.auto.paths.CloseAutoPaths;
import org.firstinspires.ftc.teamcode.revamped.utils.ArtifactColor;
import org.firstinspires.ftc.teamcode.revamped.utils.Commands.DualOptionCommand;
import org.firstinspires.ftc.teamcode.revamped.utils.Globals;

public class CloseAuto extends OpModeCommand {
    private Robot robot;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, new CloseAutoPaths());
        DecodeLimelight limelight = new DecodeLimelight(hardwareMap);
        robot.tableCompartments.populate(new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE});

        schedule(
            new Infinite(robot::update),
            new Sequential(
                    new Instant(() -> {
                        robot.flywheel.auto();
                        robot.hood.near();
                    }),
                    new Parallel(
                            robot.drivetrain.followNext(d -> d.velocityCondition(4), 3000),
                            new Race(
                                    limelight.detectMotif(),
                                    new Wait(4000)
                            )
                    ),
                    new Parallel(
                            robot.sort(),
                            new Instant(() -> robot.turret.setTargetPosition(Turret.AUTO_PRELOADS))
                    ),
                    new Instant(robot.popper::pop),
                    new Parallel(
                            new WaitUntil(robot.turret::reached),
                            new Wait(250)
                    ),
                    robot.shootAll(() -> Globals.allianceColor == null ? 0.0 : 175.0)
            )
        );
    }

    @Override
    public void end() {
        Drivetrain.startPose = robot.drivetrain.follower.getPose();
        Turret.startPos = robot.turret.getTargetPosition();
    }
}
