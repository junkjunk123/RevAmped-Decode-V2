package org.firstinspires.ftc.teamcode.mechanisms.vision;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.math.calc.Vector2D;
import org.firstinspires.ftc.teamcode.utils.ArtifactColor;
import org.firstinspires.ftc.teamcode.utils.hardware.HwDevice;

import java.util.Comparator;
import java.util.function.ToDoubleFunction;

public class DecodeHuskyLens implements HwDevice {
    private static Vector2D CROSSHAIR = new Vector2D(160, 120);
    private final HuskyLens huskyLens;
    private boolean running;
    private Vector2D target;

    private record ImageArtifact(Vector2D pos, ArtifactColor color) {
        private static Comparator<ImageArtifact> comparator() {
            Vector2D center = new Vector2D(160, 120);
            Comparator<ImageArtifact> dist = (u, v) -> Comparator.comparingDouble((ToDoubleFunction<? super ImageArtifact>) a -> a.pos.distSquared(center))
                    .compare(u, v);
            return dist.reversed();
        }
    }

    public DecodeHuskyLens(HardwareMap hardwareMap) {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        stop();
    }

    @NonNull
    @Override
    public String toString() {
        return "huskyLens";
    }

    public void stop() {
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.NONE);
        running = false;
    }

    public void start() {
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        running = true;
    }

    @Override
    public void update() {

    }
}
