package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.DecodeLimelight;
import org.firstinspires.ftc.teamcode.utils.Globals;

@Disabled
@TeleOp
public class ObeliskDetectionTest extends OpMode {
    private DecodeLimelight limelight;
    private TelemetryManager manager;

    @Override
    public void init() {
        limelight = new DecodeLimelight(hardwareMap);
        limelight.setCurrentPipeline(DecodeLimelight.Pipeline.OBELISK);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        manager = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        limelight.update();

        if (Globals.randomizationState != null) {
            telemetry.addData("motif", Globals.randomizationState);
            manager.addData("motif", Globals.randomizationState);
        } else {
            telemetry.addData("motif", "none");
            manager.addData("motif", "none");
        }

        if (limelight.getCurrentPipeline() == DecodeLimelight.Pipeline.NONE)
            limelight.setCurrentPipeline(DecodeLimelight.Pipeline.OBELISK);

        telemetry.update();
        manager.update();
    }
}
