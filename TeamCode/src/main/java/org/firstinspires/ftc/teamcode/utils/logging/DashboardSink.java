package org.firstinspires.ftc.teamcode.utils.logging;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.LinkedHashMap;
import java.util.Map;

public final class DashboardSink implements LogSink {
    private static final int MAX_TEXT_LINES = 12;

    private final FtcDashboard dashboard;
    private final DecodeLogger.Level minLevel;
    private final long updatePeriodNanos;
    private long lastUpdateNanos;
    private final Map<String, Double> numericData = new LinkedHashMap<>();
    private final Map<String, String> textData = new LinkedHashMap<>();

    public DashboardSink(DecodeLogger.Level minLevel, long updatePeriodMs) {
        this.dashboard = FtcDashboard.getInstance();
        this.minLevel = minLevel;
        this.updatePeriodNanos = Math.max(50, updatePeriodMs) * 1_000_000L;
    }

    @Override
    public String name() {
        return "DashboardSink";
    }

    @Override
    public DecodeLogger.Level minLevel() {
        return minLevel;
    }

    @Override
    public synchronized void write(LogEvent event) {
        textData.put("event/" + event.subsystem(), event.level() + " " + event.event() + " " + event.fieldsAsString());
        trimTextData();
        for (Map.Entry<String, Object> entry : event.fields().entrySet()) {
            if (entry.getValue() instanceof Number n) {
                numericData.put(
                        event.subsystem() + "." + event.event() + "." + entry.getKey(),
                        n.doubleValue()
                );
            }
        }
    }

    @Override
    public synchronized void snapshot(String tag, Map<String, Object> data) {
        String safeTag = tag == null ? "snapshot" : tag;
        for (Map.Entry<String, Object> entry : data.entrySet()) {
            Object value = entry.getValue();
            if (value instanceof Number n) {
                numericData.put(safeTag + "." + entry.getKey(), n.doubleValue());
            } else {
                textData.put("snapshot/" + safeTag + "." + entry.getKey(), String.valueOf(value));
                trimTextData();
            }
        }
    }

    @Override
    public synchronized void flush(boolean force) {
        long now = System.nanoTime();
        if (!force && now - lastUpdateNanos < updatePeriodNanos) return;

        TelemetryPacket packet = new TelemetryPacket();
        for (Map.Entry<String, Double> entry : numericData.entrySet()) {
            packet.put(entry.getKey(), entry.getValue());
        }
        for (Map.Entry<String, String> entry : textData.entrySet()) {
            packet.addLine(entry.getKey() + ": " + entry.getValue());
        }
        dashboard.sendTelemetryPacket(packet);
        lastUpdateNanos = now;
    }

    private void trimTextData() {
        while (textData.size() > MAX_TEXT_LINES) {
            String first = textData.keySet().iterator().next();
            textData.remove(first);
        }
    }
}
