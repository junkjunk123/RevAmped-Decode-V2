package org.firstinspires.ftc.teamcode.mechanisms.intake;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.HwMotor;

@Config
public class IntakeMotor extends HwMotor {
    public static float INTAKE;
    public static float OUTTAKE;
    public static float SHOOT;
    public static float STOPPED;
    public static float SHOOT_FAR;
    public static float IDLE_POWER; //to keep the balls in while moving

    public enum IntakeState {
        INTAKE,
        OUTTAKE,
        SHOOT,
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

    public void shoot(){
        setPower(SHOOT);
        state = IntakeState.SHOOT;
    }

    public void shootFar(){
        setPower(SHOOT_FAR);
        state = IntakeState.SHOOT;
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
