package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.logging.DecodeLogger;
import org.firstinspires.ftc.teamcode.utils.logging.LogProfile;
import org.firstinspires.ftc.teamcode.utils.logging.LogProfileStore;

public abstract class OpModeCommand extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LogProfile logProfile = LogProfileStore.load();
        DecodeLogger.init(telemetry, this.getClass().getSimpleName(), logProfile);
        DecodeLogger logger = DecodeLogger.get();
        boolean stopLogged = false;
        try {
            Globals.init(telemetry);
            logger.setPhase("init");
            logger.info("opmode", "OPMODE_INIT",
                    "opmode", this.getClass().getSimpleName(),
                    "alliance", Globals.allianceColor.name(),
                    "runId", logger.runId(),
                    "logProfile", logProfile.name());
            initialize();

            while (opModeInInit()) {
                telemetry.addData("loggingProfile", logProfile.name());
                initializeLoop();
                telemetry.update();
                logger.flush();
            }

            waitForStart();
            if (isStopRequested()) {
                logger.setPhase("stop");
                logger.info("opmode", "OPMODE_STOP",
                        "matchTimeSec", logger.matchTimeSec(),
                        "beforeStart", true);
                stopLogged = true;
                return;
            }

            logger.markStart();
            logger.setPhase("start");
            logger.info("opmode", "OPMODE_START", "opmode", this.getClass().getSimpleName());
            onStart();

            ElapsedTime loopTimer = new ElapsedTime();
            int loopSamplesSinceLog = 0;
            int loopDebugPeriod = logProfile.loopMsSampleEveryLoops();
            logger.setPhase("loop");
            while (opModeIsActive()) {
                loopTimer.reset();
                try {
                    execute();
                    Scheduler.getInstance().execute();
                } catch (Exception e) {
                    logger.error("opmode", "OPMODE_EXCEPTION",
                            "exception", e.getClass().getSimpleName(),
                            "message", e.getMessage());
                    logger.forceFlush();
                    throw e;
                }

                double loopMs = loopTimer.milliseconds();
                if (loopDebugPeriod > 0 && ++loopSamplesSinceLog >= loopDebugPeriod) {
                    logger.debug("opmode", "LOOP_MS",
                            "ms", loopMs,
                            "profile", logProfile.name());
                    loopSamplesSinceLog = 0;
                }
                if (loopMs > 50) {
                    logger.warn("opmode", "LOOP_SLOW", "ms", loopMs, "threshold", 50);
                }
                logger.flush();

                if (isStopRequested()) {
                    logger.setPhase("stop");
                    logger.info("opmode", "OPMODE_STOP", "matchTimeSec", logger.matchTimeSec());
                    logger.forceFlush();
                    stopLogged = true;
                    break;
                }
            }
        } finally {
            if (!stopLogged) {
                logger.setPhase("stop");
                logger.info("opmode", "OPMODE_STOP", "matchTimeSec", logger.matchTimeSec());
            }
            try {
                reset();
            } catch (Exception e) {
                logger.error("opmode", "OPMODE_EXCEPTION",
                        "hook", "reset",
                        "exception", e.getClass().getSimpleName(),
                        "message", e.getMessage());
            }
            try {
                end();
            } catch (Exception e) {
                logger.error("opmode", "OPMODE_EXCEPTION",
                        "hook", "end",
                        "exception", e.getClass().getSimpleName(),
                        "message", e.getMessage());
            }
            logger.forceFlush();
            logger.close();
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
}
