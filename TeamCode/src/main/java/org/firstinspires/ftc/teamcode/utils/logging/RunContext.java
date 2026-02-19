package org.firstinspires.ftc.teamcode.utils.logging;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public final class RunContext {
    private final long initNanos = System.nanoTime();
    private final String runId;
    private final String opModeName;
    private long startNanos = -1;
    private String phase = "init";

    public RunContext(String opModeName) {
        this.opModeName = opModeName == null ? "UnknownOpMode" : opModeName;
        String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss_SSS", Locale.US).format(new Date());
        this.runId = this.opModeName + "_" + timestamp;
    }

    public String getRunId() {
        return runId;
    }

    public String getOpModeName() {
        return opModeName;
    }

    public synchronized String getPhase() {
        return phase;
    }

    public synchronized void setPhase(String phase) {
        this.phase = phase == null || phase.isBlank() ? "unknown" : phase;
    }

    public synchronized void markStart() {
        if (startNanos < 0) {
            startNanos = System.nanoTime();
            phase = "start";
        }
    }

    public long timestampMs(long nowNanos) {
        return (nowNanos - initNanos) / 1_000_000L;
    }

    public synchronized double matchTimeSec(long nowNanos) {
        if (startNanos < 0) return 0.0;
        return (nowNanos - startNanos) / 1_000_000_000.0;
    }
}
