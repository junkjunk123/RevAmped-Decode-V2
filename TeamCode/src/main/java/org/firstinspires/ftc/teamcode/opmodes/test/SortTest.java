package org.firstinspires.ftc.teamcode.opmodes.test;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.intake.ColorManager;
import org.firstinspires.ftc.teamcode.utils.ArtifactColor;
import org.firstinspires.ftc.teamcode.utils.RandomizationState;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.utils.Types;


import java.util.List;

@Disabled
@Config
@TeleOp
public class SortTest extends OpMode {
    private ArtifactColor[] artifactColors = new ArtifactColor[3];
    private Robot robot;
    private Table table;
    public static RandomizationState motif = RandomizationState.PPG;
    private int index = -1;
    private ColorManager colorManager;


    @Override
    public void init() {
        this.robot = new Robot();
        robot = new Robot(this);
    }

    @Override
    public void start() {
        robot.update();
        List<Types.TableState> tableStates = List.of(Types.TableState.BALL1, Types.TableState.BALL2, Types.TableState.BALL3);
        artifactColors[0] = colorManager.getColorZero();
        artifactColors[2] = colorManager.getColorOne();
        artifactColors[3] = colorManager.getColorTwo();

        if (artifactColors[1] == ArtifactColor.GREEN || artifactColors[2] == ArtifactColor.GREEN)
            artifactColors[0] = ArtifactColor.PURPLE;
        else artifactColors[0] = ArtifactColor.GREEN;

        int greenIndex = motif.getGreenIndex();
        int curGreenIndex = 0;
        for (int i = 0; i < 3; i++)
            if (artifactColors[i] == ArtifactColor.GREEN)
                curGreenIndex = i;
        index = tableStates.indexOf(robot.table.getState()) - greenIndex + curGreenIndex;
        index = (index + 3) % 3;

        robot.table.getState(tableStates.get(index));
    }


    @Override
    public void loop() {

    }

    public void update() {
        robot.update();
    }

}
