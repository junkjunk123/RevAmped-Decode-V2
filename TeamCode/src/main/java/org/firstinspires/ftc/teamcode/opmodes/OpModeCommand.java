package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.Globals;

public abstract class OpModeCommand extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Globals.init(telemetry);
        initialize();
        waitForStart();

        while (opModeInInit()) {
            initializeLoop();
        }

        waitForStart();

        onStart();

        while (opModeIsActive()) {
            execute();
            Scheduler.execute();

            if (isStopRequested()) {
                reset();
                end();
            }
        }
    }

    /**
     * Cancels all previous commands
     */
    public void reset() {
        Scheduler.reset();
    }

    /**
     * Schedules objects to the scheduler
     */
    public void schedule(Command... commands) {
        Scheduler.schedule(commands);
    }

    public abstract void initialize();

    public void initializeLoop() {}

    public void execute() {}

    public void onStart() {}

    public void end() {}
}
