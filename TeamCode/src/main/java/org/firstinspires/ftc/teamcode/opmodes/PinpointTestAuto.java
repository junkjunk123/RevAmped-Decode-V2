package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.commands.Commands;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.opmodes.paths.PinpointTestPaths;
import org.firstinspires.ftc.teamcode.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.Globals;

public class PinpointTestAuto extends OpModeCommand {
    private IntakeMotor intake;
    private Flywheel flywheel;
    private Drivetrain drivetrain;
    public PinpointTestAuto(AllianceColor allianceColor) {
        Globals.allianceColor = allianceColor;
    }
    @Override
    public void initialize() {
        drivetrain = new Drivetrain(hardwareMap, new PinpointTestPaths());
        //flywheel = new Flywheel(hardwareMap);
        schedule(
                //update statements
                infinite(() ->{
                    drivetrain.update();
                    telemetry.addData("pose",drivetrain.follower.getPose());
                    telemetry.update();
                }),
//                infinite(flywheel::update),
//                infinite(intake::update),
                waitUntil(() -> !opModeInInit()),
                sequential(
                //following the actual pathing
                drivetrain.followNext(d -> d.velocityCondition(4)),
                Commands.wait(1000.0),
                drivetrain.followNext(d -> d.velocityCondition(4)),
                Commands.wait(1000.0),
                drivetrain.followNext(d -> d.velocityCondition(4)),
                drivetrain.followNext(d -> d.velocityCondition(4)),
                drivetrain.followNext(d -> d.velocityCondition(4)),
                Commands.wait(1000.0),
                drivetrain.followNext(d -> d.velocityCondition(4)),
                Commands.wait(1000.0),
                drivetrain.followNext(d -> d.velocityCondition(4)),
                Commands.wait(1000.0),
                drivetrain.followNext(d -> d.velocityCondition(4)),
                Commands.wait(1000.0),
                drivetrain.followNext(d -> d.velocityCondition(4)),
                Commands.wait(1000.0),
                drivetrain.followNext(d -> d.velocityCondition(4))
                )

        );
    }
}
