package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.logging.LogProfile;
import org.firstinspires.ftc.teamcode.utils.logging.LogProfileStore;

@TeleOp(name = "Logging Profile Selector", group = "Setup")
public class LoggingProfileSelector extends LinearOpMode {
    @Override
    public void runOpMode() {
        LogProfile[] profiles = LogProfile.values();
        LogProfile savedProfile = LogProfileStore.load();
        int selectedIndex = savedProfile.ordinal();
        String status = "";

        boolean prevLeft = false;
        boolean prevRight = false;
        boolean prevA = false;

        while (opModeInInit() && !isStopRequested()) {
            boolean left = gamepad1.dpad_left;
            boolean right = gamepad1.dpad_right;
            boolean a = gamepad1.a;

            if (left && !prevLeft) {
                selectedIndex = (selectedIndex - 1 + profiles.length) % profiles.length;
            }
            if (right && !prevRight) {
                selectedIndex = (selectedIndex + 1) % profiles.length;
            }
            if (a && !prevA) {
                LogProfile selected = profiles[selectedIndex];
                if (LogProfileStore.save(selected)) {
                    savedProfile = selected;
                    status = "Saved profile: " + selected.name();
                } else {
                    status = "Save failed";
                }
            }

            telemetry.addLine("Logging Profile Selector");
            telemetry.addData("Saved", savedProfile.name());
            telemetry.addData("Selected", profiles[selectedIndex].name());
            telemetry.addLine("Profiles: MATCH | PRACTICE | DEBUG");
            telemetry.addLine("Controls: D-pad left/right = select");
            telemetry.addLine("Controls: A = save, STOP = exit");
            if (!status.isEmpty()) telemetry.addData("Status", status);
            telemetry.update();

            prevLeft = left;
            prevRight = right;
            prevA = a;
            idle();
        }
    }
}
