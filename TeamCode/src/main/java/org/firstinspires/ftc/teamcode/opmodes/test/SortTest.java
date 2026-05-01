package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.ArtifactColor;
import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Lazy;
import org.firstinspires.ftc.teamcode.utils.commands.RandomizationState;

@TeleOp
public class SortTest extends OpMode {
    private ArtifactColor[] artifactColors = new ArtifactColor[3];
    private Robot robot;
    public static RandomizationState motif = RandomizationState.PPG;
    private int index = -1;

    @Override
    public void init() {
        Globals.init(telemetry);
        robot = new Robot(hardwareMap);
    }

    @Override
    public void start() {
        robot.update();
        Globals.randomizationState = motif;
        Scheduler.getInstance().schedule(
                new Lazy(() -> {
                    if (Globals.randomizationState != null) {
                        return new Sequential(
                                new Race(
                                        robot.tableCompartments.populateAuto(),
                                        new Wait(500)
                                ),
                                robot.sortAuto()
                        );
                    }
                    return Commands.NOOP;
                })
        );
    }


    @Override
    public void loop() {
        Scheduler.getInstance().execute();
        telemetry.addData("val", robot.tableCompartments.sort());
        telemetry.update();
    }
}
