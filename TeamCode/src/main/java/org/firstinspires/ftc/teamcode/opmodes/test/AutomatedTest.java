package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.utils.commands.channel.Channel;
import org.firstinspires.ftc.teamcode.utils.commands.channel.Channels;

import java.util.concurrent.atomic.AtomicReference;

//@Disabled
@TeleOp
public class AutomatedTest extends OpModeCommand {
    @Override
    public void initialize() {
        Robot robot = new Robot(hardwareMap);
        Channel<String> channel = Channels.stream();
        AtomicReference<Pose> lastPose = new AtomicReference<>();

        schedule(
                new Instant(robot.hood::near),
                new Parallel(
                        robot.intake.intakeMotor.test().subscribe(channel),
                        robot.flywheel.test().subscribe(channel)
                ),
                new Infinite(() -> {
                    int i = -1;
                    for (String line : channel.getStream()) {
                        i++;
                        telemetry.addData(String.valueOf(i), line);
                    }

                    robot.drivetrain.arcadeDrive(gamepad1);
                    robot.drivetrain.follower.poseTracker.getLocalizer().update();

                    if (gamepad1.aWasPressed()) {
                        schedule(
                                new Sequential(
                                        new Instant(() -> lastPose.set(robot.drivetrain.follower.getPose())),
                                        new Wait(2000),
                                        Channels.send(channel, () -> {
                                            if (lastPose.get().distanceFrom(robot.drivetrain.follower.getPose()) > 2) {
                                                return "Drivetrain subsystem: passed";
                                            }

                                            return "Drivetrain subsystem: failed";
                                        })
                                )
                        );
                    }

                    telemetry.update();
                })
        );
    }

    @Override
    public void initializeLoop() {
        telemetry.addData("Automated Test", "Press play to begin");
        telemetry.update();
    }
}
