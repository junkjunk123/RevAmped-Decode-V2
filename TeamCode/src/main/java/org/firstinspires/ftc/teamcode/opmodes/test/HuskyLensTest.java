package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.dfrobot.HuskyLens.Block;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.commands.ArtifactColor;
import org.firstinspires.ftc.teamcode.utils.math.calc.Vector2D;

import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.function.ToDoubleFunction;

@Disabled
@TeleOp
public class HuskyLensTest extends OpMode {
    private HuskyLens camera;

    @Override
    public void init() {
        camera = hardwareMap.get(HuskyLens.class, "huskylens");
        camera.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    @Override
    public void init_loop() {
        Block[] blocks = camera.blocks();
        int green = 0;
        int purple = 0;

        for (Block block : blocks) {
            if (block.height < 0.75 * block.width)
                continue;
            if (block.width < 0.75 * block.height)
                continue;

            ArtifactColor color = block.id == 1 ?
                    ArtifactColor.GREEN : ArtifactColor.PURPLE;

            if (color == ArtifactColor.GREEN) {
                green++;
                telemetry.addData(color.name() + " " + green, new Vector2D(block.x, block.y));
            } else {
                purple++;
                telemetry.addData(color.name() + " " + purple, new Vector2D(block.x, block.y));
            }
        }

        telemetry.addData("total", green + purple);
        telemetry.update();
    }

    @Override
    public void loop() {
        Block[] blocks = camera.blocks();
        PriorityQueue<Artifact> artifactPQ = new PriorityQueue<>(Artifact.comparator());
        int green = 0;
        int purple = 0;

        for (Block block : blocks) {
            if (block.height < 0.75 * block.width)
                continue;
            if (block.width < 0.75 * block.height)
                continue;

            ArtifactColor color = block.id == 1 ?
                    ArtifactColor.GREEN : ArtifactColor.PURPLE;

            if (color == ArtifactColor.GREEN) {
                green++;
            } else {
                purple++;
            }

            artifactPQ.offer(new Artifact(new Vector2D(block.x, block.y), color));
        }

        for (int i = 0; i < 3; i++)
            telemetry.addData(String.valueOf(i), artifactPQ.poll());

        telemetry.addData("totalGreen", green);
        telemetry.addData("totalPurple", purple);
        telemetry.update();
    }

    private record Artifact(Vector2D pos, ArtifactColor color) {
        private static Comparator<Artifact> comparator() {
            Vector2D center = new Vector2D(160, 120);
            Comparator<Artifact> dist = (u, v) -> Comparator.comparingDouble((ToDoubleFunction<? super Artifact>) a -> a.pos.distSquared(center))
                    .compare(u, v);
            return dist.reversed();
        }
    }
}
