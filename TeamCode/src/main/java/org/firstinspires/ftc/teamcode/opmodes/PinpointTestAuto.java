package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.commands.Commands;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.octocanum.Octocanum;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.opmodes.paths.FarAutoPaths;
import org.firstinspires.ftc.teamcode.opmodes.paths.PinpointTestPaths;
import org.firstinspires.ftc.teamcode.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.Globals;

public class PinpointTestAuto extends OpModeCommand {
    private IntakeMotor intake;
    private Flywheel flywheel;
    private Drivetrain drivetrain;
    private Octocanum octocanum;
    public PinpointTestAuto(AllianceColor allianceColor) {
        Globals.allianceColor = allianceColor;
    }
    @Override
    public void initialize() {
        drivetrain = new Drivetrain(hardwareMap, new PinpointTestPaths());
        octocanum = new Octocanum(hardwareMap);
        //flywheel = new Flywheel(hardwareMap);
        schedule(
                //update statements
                infinite(() ->{
                    drivetrain.update();
                    telemetry.addData("pose",drivetrain.follower.getPose());
                    telemetry.update();
                }),
//                new Infinite(flywheel::update),
//                new Infinite(intake::update),
                waitUntil(() -> !opModeInInit()),
                sequential(
                //following the actual pathing
                drivetrain.followNext(d -> d.velocityCondition(4)),
                parallel(
                waitMs(1000.0),
                        instant(() -> octocanum.engage())
                        ),
                drivetrain.followNext(d -> d.velocityCondition(4)),
                parallel(
                waitMs(1000.0),
                        instant(() -> octocanum.raise())
                        ),
                drivetrain.followNext(d -> d.velocityCondition(4)),
                        waitMs(1000.0),
                drivetrain.followNext(d -> d.velocityCondition(4)),
                        waitMs(1000.0),
                drivetrain.followNext(d -> d.velocityCondition(4)),
                waitMs(1000.0),
                drivetrain.followNext(d -> d.velocityCondition(4)),
                waitMs(1000.0),
                drivetrain.followNext(d -> d.velocityCondition(4)),
                waitMs(1000.0),
                drivetrain.followNext(d -> d.velocityCondition(4)),
                waitMs(1000.0),
                drivetrain.followNext(d -> d.velocityCondition(4)),
                waitMs(1000.0),
                drivetrain.followNext(d -> d.velocityCondition(4))
                )
        );
    }
}
