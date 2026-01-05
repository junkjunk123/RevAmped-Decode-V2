package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Sequential;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.opmodes.paths.FarAutoPaths;
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
                new Infinite(() ->{
                    drivetrain.update();
                    telemetry.addData("pose",drivetrain.follower.getPose());
                    telemetry.update();
                }),
//                new Infinite(flywheel::update),
//                new Infinite(intake::update),
                new WaitUntil(() -> !opModeInInit()),
                new Sequential(
                //following the actual pathing
                drivetrain.followNext(d -> d.velocityCondition(4)),
                new Wait(1000),
                drivetrain.followNext(d -> d.velocityCondition(4)),
                new Wait(1000),
                drivetrain.followNext(d -> d.velocityCondition(4)),
                drivetrain.followNext(d -> d.velocityCondition(4)),
                drivetrain.followNext(d -> d.velocityCondition(4)),
                new Wait(1000),
                drivetrain.followNext(d -> d.velocityCondition(4)),
                new Wait(1000),
                drivetrain.followNext(d -> d.velocityCondition(4)),
                new Wait(1000),
                drivetrain.followNext(d -> d.velocityCondition(4)),
                new Wait(1000),
                drivetrain.followNext(d -> d.velocityCondition(4)),
                new Wait(1000),
                drivetrain.followNext(d -> d.velocityCondition(4))
                )

        );
    }
}
