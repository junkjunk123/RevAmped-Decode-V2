package org.firstinspires.ftc.teamcode.utils.logging;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedHashMap;
import java.util.Map;

public final class TelemetrySink implements LogSink {
    private static final int MAX_SNAPSHOT_FIELDS = 36;

    private final Telemetry telemetry;
    private final DecodeLogger.Level minLevel;
    private final long updatePeriodNanos;
    private long lastUpdateNanos;
    private String lastEventLine = "";
    private String lastWarnLine = "";
    private final Map<String, Object> snapshotData = new LinkedHashMap<>();

    public TelemetrySink(Telemetry telemetry, DecodeLogger.Level minLevel, long updatePeriodMs) {
        this.telemetry = telemetry;
        this.minLevel = minLevel;
        this.updatePeriodNanos = Math.max(50, updatePeriodMs) * 1_000_000L;
    }

    @Override
    public String name() {
        return "TelemetrySink";
    }

    @Override
    public DecodeLogger.Level minLevel() {
        return minLevel;
    }

    @Override
    public synchronized void write(LogEvent event) {
        String line = event.subsystem() + ":" + event.event() + " " + event.fieldsAsString();
        if (event.level().ordinal() >= DecodeLogger.Level.WARN.ordinal()) {
            lastWarnLine = event.level().name() + " " + line;
        }
        lastEventLine = line;
    }

    @Override
    public synchronized void snapshot(String tag, Map<String, Object> data) {
        String safeTag = tag == null ? "snapshot" : tag;
        for (Map.Entry<String, Object> entry : data.entrySet()) {
            snapshotData.put(safeTag + "." + entry.getKey(), entry.getValue());
            trimSnapshots();
        }
    }

    @Override
    public synchronized void flush(boolean force) {
        long now = System.nanoTime();
        if (!force && now - lastUpdateNanos < updatePeriodNanos) return;

        if (!lastWarnLine.isEmpty()) {
            telemetry.addData("log/ALERT", lastWarnLine);
        }
        if (!lastEventLine.isEmpty()) {
            telemetry.addData("log/event", lastEventLine);
        }
        for (Map.Entry<String, Object> entry : snapshotData.entrySet()) {
            telemetry.addData("log/" + entry.getKey(), String.valueOf(entry.getValue()));
        }
        telemetry.update();
        lastUpdateNanos = now;
    }

    private void trimSnapshots() {
        while (snapshotData.size() > MAX_SNAPSHOT_FIELDS) {
            String first = snapshotData.keySet().iterator().next();
            snapshotData.remove(first);
        }
    }
}
