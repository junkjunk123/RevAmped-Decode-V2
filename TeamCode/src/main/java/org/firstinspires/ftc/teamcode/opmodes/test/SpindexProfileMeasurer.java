package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.utils.hardware.Encoder;

@TeleOp
public class SpindexProfileMeasurer extends OpModeCommand {
    private Table table;
    private DcMotorEx encoderMotor;

    @Override
    public void initialize() {
        encoderMotor = hardwareMap.get(DcMotorEx.class, "motor_rb");
        table = new Table(hardwareMap, Encoder.fromMotor(encoderMotor));
        table.setPosition(Table.BALL1);

        schedule(
                new Infinite(table::update),
                new Instant(() -> {
                    table.shoot();
                })
        );
    }
}
