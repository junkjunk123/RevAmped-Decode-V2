package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.groups.Parallel;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.utils.commands.channel.Channel;
import org.firstinspires.ftc.teamcode.utils.commands.channel.Channels;

public class AutomatedTest extends OpModeCommand {
    @Override
    public void initialize() {
        Robot robot = new Robot(hardwareMap);
        Channel<String> channel = Channels.stream();

        schedule(
                robot.popper.pop(),
                robot.popper.neutral(),
                new Instant(robot.hood::near),
                new Parallel(
                        robot.intakeMotor.test().subscribe(channel),
                        robot.turret.test().subscribe(channel),
                        robot.table.test().subscribe(channel),
                        robot.flywheel.test().subscribe(channel)
                )
        );
    }
}
