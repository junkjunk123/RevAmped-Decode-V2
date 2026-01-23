package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Sequential;
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
                new Infinite(robot::update),
                new Sequential(
                        new Instant(() -> {
                            robot.flywheel.far();
                            robot.hood.far();
                        }),
                        new Parallel(
                                robot.popper.pop(),
                                robot.turret.runToPos(Turret.FAR_AUTO),
                                new Wait(750)
                        ),
                        robot.shootAll(100),
                        new Parallel(
                                robot.resetTable(),
                                new Instant(() -> robot.flywheel.stop()),
                                robot.drivetrain.followNext(d -> d.velocityCondition(4), 1500)
                        ),
                        robot.drivetrain.followNext(d -> d.velocityCondition(4), 2500),
                        new Instant(robot.intakeMotor::intake),
                        new Wait(500),
                        robot.drivetrain.followNext(d -> d.velocityCondition(4), 1500),
                        new Instant(() -> robot.flywheel.far()),
                        robot.drivetrain.followNext(d -> d.velocityCondition(4), 1500),
                        new Parallel(
                                robot.drivetrain.followNext(d -> d.velocityCondition(4), 2500),
                                new Sequential(
                                        new Wait(500),
                                        robot.popper.pop()
                                )
                        )
                )
        );
    }
}
