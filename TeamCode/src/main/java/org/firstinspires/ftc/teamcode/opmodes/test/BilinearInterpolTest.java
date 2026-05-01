package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.math.Matrix;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import smile.interpolation.BilinearInterpolation;

@Disabled
@TeleOp
public class BilinearInterpolTest extends OpMode {
    double[] X = new double[] {0, 0.25, 0.5, 0.75};
    double[] Y = new double[] {0, 5, 10, 15};
    double[][] interpolation = new double[][] {
            {0, 200, 400, 600},
            {10, 210, 410, 610},
            {20, 220, 420, 620},
            {30, 230, 430, 630}
    };
    BilinearInterpolation interpol;
    double x = 0, y = 0;

    @Override
    public void init() {
        interpolation = new Matrix(interpolation).transposed().getMatrix();
        interpol = new BilinearInterpolation(X, Y, interpolation);
    }

    @Override
    public void loop() {
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("z", interpol.interpolate(x, y));

        if (gamepad1.aWasPressed()) {
            x+=0.25;
        }

        if (gamepad1.xWasPressed()) {
            x-=0.25;
        }

        if (gamepad1.bWasPressed()) {
            y+=5;
        }

        if (gamepad1.yWasPressed()) {
            y-=5;
        }

        telemetry.update();
    }
}
