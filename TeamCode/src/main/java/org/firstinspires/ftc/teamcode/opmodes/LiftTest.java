package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.hardware.HwMotor;
import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;

@TeleOp
public class LiftTest extends OpModeCommand{
    //LIFT
    //GOES POSITIVE
    //LIFT 2 GOES NEGATIVE
    private HwServo lift_1;
    private HwServo lift_2;
    @Override
    public void initialize() {
        lift_1 = new HwServo(hardwareMap,"lift_1");
        lift_2 = new HwServo(hardwareMap,"lift_2");
        lift_1.setPosition(255/255f);
        lift_2.setPosition(0/255f);
    }
}
