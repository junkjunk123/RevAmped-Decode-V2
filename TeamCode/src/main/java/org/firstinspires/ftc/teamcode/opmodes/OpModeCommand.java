package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;

public abstract class OpModeCommand extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        preInit();
        Globals.init(telemetry);
        initialize();

        while (opModeInInit()) {
            initializeLoop();
        }

        waitForStart();

        onStart();

        while (opModeIsActive()) {
            execute();
            Scheduler.getInstance().execute();

            if (isStopRequested()) {
                reset();
                if (Robot.INSTANCE != null) Robot.INSTANCE.close();
                end();
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

    public void initializeLoop() {}

    public void execute() {}

    public void onStart() {}

    public void end() {}

    public void preInit() {}
}
