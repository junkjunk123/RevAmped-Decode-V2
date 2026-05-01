package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.ArtifactColor;
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
        robot.tableCompartments.populate(ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE);
    }

    @Override
    public void start() {
        robot.update();
        Globals.randomizationState = motif;
        index = robot.tableCompartments.sort();
        robot.table.setPosition(Table.RelativeState.values()[(index + 2) % 3].target());
    }


    @Override
    public void loop() {
        telemetry.addData("val", robot.tableCompartments.sort());
        telemetry.update();
    }
}
