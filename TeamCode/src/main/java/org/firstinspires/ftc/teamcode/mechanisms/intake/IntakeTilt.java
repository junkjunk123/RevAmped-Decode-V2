package org.firstinspires.ftc.teamcode.mechanisms.intake;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;

public class IntakeTilt extends HwServo {
    public static double INTAKE;
    public static double TRANSFER;

    private enum TiltState {
        INTAKE,
        TRANSFER
    }
    private TiltState state = TiltState.INTAKE;

    public IntakeTilt(HardwareMap hardwareMap) {
        super(hardwareMap, "intakeTilt");
    }

    public void intake() {
        setPosition(INTAKE);
        state = TiltState.INTAKE;
    }

    public void transfer() {
        setPosition(TRANSFER);
        state = TiltState.TRANSFER;
    }
}
