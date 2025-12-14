package org.firstinspires.ftc.teamcode.revamped;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.revamped.utils.PathSupplier;

public class Robot {
    public static Robot INSTANCE;
    public final Drivetrain drivetrain;

    public Robot(HardwareMap hardwareMap) {
        drivetrain = new Drivetrain(hardwareMap);
    }

    public Robot(HardwareMap hardwareMap, PathSupplier pathSupplier) {
        drivetrain = new Drivetrain(hardwareMap, pathSupplier);
    }

    public void update() {
        drivetrain.update();
    }
}
