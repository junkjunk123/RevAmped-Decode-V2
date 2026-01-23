package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.utils.commands.channel.Channel;
import org.firstinspires.ftc.teamcode.utils.commands.channel.Channels;
import org.firstinspires.ftc.teamcode.utils.commands.channel.Notifier;

public class UnsortedCloseAuto extends OpModeCommand {
    private Robot robot;
    private final ElapsedTime overallTimer = new ElapsedTime();
    private final Channel<Object> notificationStream = Channels.stream();

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);

        schedule(
                new Infinite(() -> {
                    robot.update();
                }),
                new Sequential(
                        new WaitUntil(() -> !opModeInInit()),
                        new Instant(overallTimer::reset)
                ),
                new Sequential(
                        new Instant(() -> {
                            robot.flywheel.closeAuto();
                            robot.hood.near();
                            robot.turret.move(Turret.MoveState.CLOSE_AUTO);
                        }),
                        robot.drivetrain.followNext(f -> f.velocityCondition(4), 3000),
                        shootAndIntake(0),
                        shootAndIntake(1),
                        shootAndIntake(2),
                        shootAndIntake(3)
                )
        );
    }

    private Notifier shoot() {
        return new Notifier(c -> new Sequential(
                robot.popper.pop(),
                robot.shootAll(),
                Channels.send(c, Channels::signal),
                robot.resetTable()
        ));
    }

    private ICommand shootAndIntake(int iteration) {
        return new Sequential(
                new Parallel(
                        shoot().subscribe(notificationStream),
                        new Sequential(
                                notificationStream.listenAndClear(),
                                robot.drivetrain.followNext(f -> f.velocityCondition(4),
                                        getIntakeTimeout(iteration))
                        )
                ),
                waitForIntake(iteration)
        );
    }

    private ICommand waitForIntake(int iteration) {
        //TODO: Code
        return new Wait(0);
    }

    public int getIntakeTimeout(int iteration) {
        return iteration == 2 ? 4500 : 3000;
    }

    public int getShootTimeout(int iteration) {
        return switch (iteration) {
            case 0 -> 3000;
            case 1 -> 2700;
            default -> 3500;
        };
    }
}
