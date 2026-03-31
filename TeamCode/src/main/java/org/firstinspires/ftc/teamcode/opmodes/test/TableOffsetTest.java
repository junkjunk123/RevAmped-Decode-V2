package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class TableOffsetTest extends OpMode {
    private Servo table;
    private Servo table2;

    @Override
    public void init() {
        table = hardwareMap.get(ServoImplEx.class, "table");
        table2 = hardwareMap.get(ServoImplEx.class, "table2");
    }

    @Override
    public void loop() {

    }
}
