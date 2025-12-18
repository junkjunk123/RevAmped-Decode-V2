package org.firstinspires.ftc.teamcode.revamped.opmodes;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.revamped.utils.Globals;

public abstract class OpModeCommand extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Globals.init(telemetry);
        waitForStart();

        while (opModeInInit()) {
            initLoop();
        }

        waitForStart();

        onStart();

        while (opModeIsActive()) {
            update();
            Scheduler.getInstance().execute();

            if (isStopRequested()) {
                reset();
                onStop();
            }
        }
    }

    /**
     * Cancels all previous commands
     */
    public void reset() {
        Scheduler.getInstance().reset();
    }

    /**
     * Schedules objects to the scheduler
     */
    public void schedule(ICommand... commands) {
        Scheduler.getInstance().schedule(commands);
    }

    public abstract void initialize();

    public void initLoop() {}

    public void update() {}

    public void onStart() {}

    public void onStop() {}
}
