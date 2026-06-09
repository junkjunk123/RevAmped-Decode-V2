package org.firstinspires.ftc.teamcode.mechanisms.intake;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.HwMotor;

public class IntakeMotor extends HwMotor {
    public static float INTAKE;
    public static float OUTTAKE;
    public static float STOPPED;
    public static float TRANSFER_FAR;
    public static float IDLE_POWER; //to keep the balls in while moving

    public enum IntakeState {
        INTAKE,
        OUTTAKE,
        TRANSFER_FAR,
        IDLE,
        STOPPED
    }
    private IntakeState state = IntakeState.STOPPED;

    public IntakeMotor(HardwareMap hardwareMap) {
        super(hardwareMap, "intake");
        setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void intake() {
        setPower(INTAKE);
        state = IntakeState.INTAKE;
    }

    public void idle(){
        setPower(IDLE_POWER);
        state = IntakeState.IDLE;
    }

    public void transferFar(){
        setPower(TRANSFER_FAR);
        state = IntakeState.TRANSFER_FAR;
    }

    public void outtake() {
        setPower(OUTTAKE);
        state = IntakeState.OUTTAKE;
    }

    public void stop() {
        setPower(STOPPED);
        state = IntakeState.STOPPED;
    }

    public String getState() {
        return state.name();
    }

    public boolean atState(IntakeState state) {
        return state == this.state;
    }
}
