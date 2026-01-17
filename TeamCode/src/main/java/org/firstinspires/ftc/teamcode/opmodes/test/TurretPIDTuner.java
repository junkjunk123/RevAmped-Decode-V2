package org.firstinspires.ftc.teamcode.opmodes.test;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class TurretPIDTuner extends OpMode {
    private DcMotorEx turret;
    public static int targetPosition;
    private int startPos;
    public static double P = 0.01;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0.01;
    private PIDFController controller;

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        startPos = turret.getCurrentPosition();
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        controller = new PIDFController(new PIDFCoefficients(P, I, D, F));
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void init_loop() {
        telemetry.addData("pos", pos());
        telemetry.update();
    }

    private int pos() {
        return turret.getCurrentPosition() - startPos;
    }

    public void loop() {
        controller.setCoefficients(new PIDFCoefficients(P, I, D, F));
        int error = targetPosition - pos();
        controller.updateFeedForwardInput(Math.signum(error));
        controller.updateError(error);
        double power = controller.run();
        turret.setPower(power);
        telemetry.addData("power",power);
        telemetry.addData("currentPos",pos());
        telemetry.addData("error",targetPosition - pos());
        telemetry.update();
    }
}
