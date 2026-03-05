package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.GamepadEx;
import org.firstinspires.ftc.teamcode.utils.hardware.HwMotor;

@Disabled
@TeleOp
public class MotorPortFinder extends OpModeCommand{
    GamepadEx gamepad_1;
    private HwMotor port0;
    private HwMotor port1;
    private HwMotor port2;
    private HwMotor port3;

    @Override
    public void initialize() {
        port0 = new HwMotor(hardwareMap,"port0");
        port1 = new HwMotor(hardwareMap,"port1");
        port2 = new HwMotor(hardwareMap,"port2");
        port3 = new HwMotor(hardwareMap,"port3");
        gamepad_1 = new GamepadEx(gamepad1);
        schedule(
                new Infinite(()->{
                    port0.update();
                    port1.update();
                    port2.update();
                    port3.update();
                }
                )
        );
    }

    @Override
    public void execute(){
        gamepad_1.update();
        if(gamepad_1.y.isRisingEdge()) schedule(new Instant(() -> port0.setPower(port0.getPower()!=0 ? 0f : 0.5f)));
        if(gamepad_1.b.isRisingEdge()) schedule(new Instant(() -> port1.setPower(port1.getPower()!=0 ? 0f : 0.5f)));
        if(gamepad_1.a.isRisingEdge()) schedule(new Instant(() -> port2.setPower(port2.getPower()!=0 ? 0f : 0.5f)));
        if(gamepad_1.x.isRisingEdge()) schedule(new Instant(() -> port3.setPower(port3.getPower()!=0 ? 0f : 0.5f)));
    }
}
