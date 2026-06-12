package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.math.projectile.HoodInverseKinematics;

@TeleOp
public class HoodInverseKinematicsTester extends OpMode {
    private HoodInverseKinematics inverseKinematics;

    @Override
    public void init() {
        inverseKinematics = new HoodInverseKinematics();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        inverseKinematics.calculateHoodAngle();
        telemetry.addData("calculated angle rad",inverseKinematics.getAngle());
        telemetry.addData("calculated angle deg",Math.toDegrees(inverseKinematics.getAngle()));
        telemetry.update();
    }
}
