package org.firstinspires.ftc.teamcode.opmodes.test;

import static org.firstinspires.ftc.teamcode.mechanisms.intake.Table.FULL_REVOLUTION;
import static org.firstinspires.ftc.teamcode.mechanisms.intake.Table.MS_PER_REVOLUTION;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.hardware.Encoder;

@TeleOp
public class TableEncoderTest extends OpMode {
    private Table table;
    private IntakeMotor intakeMotor;
    private double previousPos = Table.BALL1;
    private double distance;
    private double actualTime;
    private ElapsedTime timer = new ElapsedTime();
    private boolean stop = true;

    @Override
    public void init() {
        intakeMotor = new IntakeMotor(hardwareMap);
        table = new Table(hardwareMap, Encoder.external(intakeMotor.get()));
        Globals.init(telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            table.setPosition(Table.BALL1_END);
            distance = Math.abs(Table.BALL1_END - previousPos);
            previousPos = Table.BALL1_END;
            timer.reset();
            stop = false;
        }

        if (gamepad1.bWasPressed()) {
            table.setPosition(Table.RelativeState.BALL1.target());
            distance = Math.abs(Table.BALL1 - previousPos);
            previousPos = Table.BALL1;
            timer.reset();
            stop = false;
        }

        if (timer.milliseconds() > 250 && table.getEncoder().getVelocity() < 10 && !stop) {
            actualTime = timer.milliseconds();
            stop = true;
        }

        telemetry.addData("tableVel", table.getEncoder().getVelocity());
        telemetry.addData("tablePos", table.getEncoder().getPosition());
        telemetry.addData("dist", distance);
        telemetry.addData("time", Math.abs(distance / FULL_REVOLUTION * MS_PER_REVOLUTION));
        telemetry.addData("actualTime", actualTime);
        telemetry.update();
    }
}
