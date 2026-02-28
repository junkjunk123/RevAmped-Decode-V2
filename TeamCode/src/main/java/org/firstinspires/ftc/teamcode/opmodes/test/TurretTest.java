package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.hardware.Encoder;

@Disabled
@TeleOp
public class TurretTest extends OpMode {
    private Turret turret;

    @Override
    public void init() {
        Globals.constants.build();
        turret = new Turret(hardwareMap, Encoder.fromMotor(hardwareMap.get(DcMotorEx.class, "motor_lf")));
    }

    @Override
    public void init_loop() {
        telemetry.addData("pos", turret.getEncoder().getPosition());
        telemetry.addData("encoderBase", turret.getEncoderBase());
        telemetry.addData("target", turret.getTargetPosition());
        telemetry.addData("currentPos", turret.getPosition());
        telemetry.update();
    }

    @Override
    public void loop() {
        turret.getController().setCoefficients(new PIDFCoefficients(Turret.P, Turret.I, Turret.D, Turret.F));
        turret.getSecondaryController().setCoefficients(
                new PIDFCoefficients(Turret.P_SECONDARY, Turret.I_SECONDARY, Turret.D_SECONDARY, Turret.F_SECONDARY));
        telemetry.addData("current", turret.get().getCurrentPosition());
        telemetry.addData("base", turret.getEncoderBase());
        telemetry.addData("pos", turret.getPosition());
        telemetry.update();
        turret.update();

        if (gamepad1.aWasPressed()) {
            turret.setTargetPosition(500);
        }

        if (gamepad1.bWasPressed()) {
            turret.setTargetPosition(0);
        }

        if (gamepad1.xWasPressed()) {
            turret.setTargetPosition(-250);
        }
    }
}
