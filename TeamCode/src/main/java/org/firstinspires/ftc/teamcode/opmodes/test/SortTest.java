package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.utils.ArtifactColor;
import org.firstinspires.ftc.teamcode.utils.RandomizationState;

@Disabled
@Config
@TeleOp
public class SortTest extends OpMode {
    private ArtifactColor[] artifactColors = new ArtifactColor[3];
    private Robot robot;
    public static RandomizationState motif = RandomizationState.PPG;
    private int index = -1;


    @Override
    public void init() {
        robot = new Robot(hardwareMap);
    }

    @Override
    public void start() {
        robot.update();

        if (artifactColors[1] == ArtifactColor.GREEN || artifactColors[2] == ArtifactColor.GREEN)
            artifactColors[0] = ArtifactColor.PURPLE;
        else artifactColors[0] = ArtifactColor.GREEN;

        int greenIndex = motif.getGreenIndex();
        int curGreenIndex = 0;
        for (int i = 0; i < 3; i++)
            if (artifactColors[i] == ArtifactColor.GREEN)
                curGreenIndex = i;
        index = robot.table.getState().ordinal() - greenIndex + curGreenIndex;
        index = (index + 3) % 3;
        robot.table.setPosition(Table.RelativeState.values()[index].target());
    }


    @Override
    public void loop() {
        update();
    }

    public void update() {
        robot.update();
    }
}
