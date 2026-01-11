package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.opmodes.paths.FarAutoPaths;
import org.firstinspires.ftc.teamcode.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.Globals;

public class FarAuto extends OpModeCommand {
    private Robot robot;
    private ElapsedTime overallTimer = new ElapsedTime();

    public FarAuto(AllianceColor allianceColor) {
        Globals.allianceColor = allianceColor;
    }

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, new FarAutoPaths());

        schedule(
                infinite(robot::update),
                sequential(
                        instant(() -> {
                            robot.flywheel.far();
                            robot.hood.far();
                        }),
                        parallel(
                                robot.popper.pop(),
                                robot.turret.runToPos(Turret.FAR_AUTO),
                                Commands.wait(750.0)
                        ),
                        robot.shootAll(100),
                        parallel(
                                robot.resetTableAfterShooting(),
                                instant(() -> robot.flywheel.stop()),
                                robot.drivetrain.followNext(d -> d.velocityCondition(4), 1500)
                        ),
                        robot.drivetrain.followNext(d -> d.velocityCondition(4), 2500),
                        instant(robot.intakeMotor::intake),
                        Commands.wait(500.0),
                        robot.drivetrain.followNext(d -> d.velocityCondition(4), 1500),
                        instant(() -> robot.flywheel.far()),
                        robot.drivetrain.followNext(d -> d.velocityCondition(4), 1500),
                        parallel(
                                robot.drivetrain.followNext(d -> d.velocityCondition(4), 2500),
                                sequential(
                                        Commands.wait(500.0),
                                        robot.popper.pop()
                                )
                        )
                )
        );
    }
}
