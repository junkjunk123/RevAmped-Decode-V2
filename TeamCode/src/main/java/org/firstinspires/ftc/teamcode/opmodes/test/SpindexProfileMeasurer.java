package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Deadline;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.utils.commands.Lazy;
import org.firstinspires.ftc.teamcode.utils.math.calc.Differentiator;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Disabled
@TeleOp
public class SpindexProfileMeasurer extends OpModeCommand {
    private Robot robot;
    private Differentiator differentiator;
    private final List<Double> accels = new ArrayList<>();

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        robot.feederWheel.start();
        robot.popper.popCommandless();
        differentiator = new Differentiator(() -> 0.0, () -> Math.abs(robot.table.getEncoder().getVelocity()));

        AtomicReference<Double> lastAccel = new AtomicReference<>();

        schedule(
                new Infinite(() -> {
                    robot.update();

                    if (!accels.isEmpty())
                        telemetry.addData("average accel", (accels.stream().mapToDouble(Double::doubleValue).sum()) / accels.size());

                    for (int i = 0; i < accels.size(); i++) {
                        telemetry.addData(String.valueOf(i), accels.get(i));
                    }
                }),
                new Instant(() -> {
                    robot.intakeTilt.intake();
                    robot.intakeGate.open();
                }),
                new Deadline(
                        new Lazy(() -> {
                            float pos = switch (robot.table.getState()) {
                                case BALL0 -> Table.BALL0_END;
                                case BALL1 -> Table.BALL1_END;
                                case BALL2 -> Table.BALL2_END;
                            };

                            return new Sequential(
                                    new Instant(() -> robot.table.setPosition(pos)),
                                    new Wait(500)
                            );
                        }),
                        new Sequential(
                                new Race(
                                        new Infinite(() -> {
                                            differentiator.update();
                                            lastAccel.set(differentiator.calculate());
                                        }),
                                        new WaitUntil(() -> lastAccel.get() < 0)
                                ),
                                new Infinite(() -> {
                                    differentiator.update();
                                    accels.add(differentiator.calculate());
                                })
                        )
                )
        );
    }
}
