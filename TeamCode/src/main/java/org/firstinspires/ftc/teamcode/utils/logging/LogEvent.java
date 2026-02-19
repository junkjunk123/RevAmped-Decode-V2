package org.firstinspires.ftc.teamcode.utils.logging;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;
import java.util.StringJoiner;

public final class LogEvent {
    private final long timestampMs;
    private final double matchTimeSec;
    private final String runId;
    private final String opMode;
    private final String phase;
    private final DecodeLogger.Level level;
    private final String subsystem;
    private final String event;
    private final Map<String, Object> fields;

    public LogEvent(
            long timestampMs,
            double matchTimeSec,
            String runId,
            String opMode,
            String phase,
            DecodeLogger.Level level,
            String subsystem,
            String event,
            Map<String, Object> fields
    ) {
        this.timestampMs = timestampMs;
        this.matchTimeSec = matchTimeSec;
        this.runId = runId;
        this.opMode = opMode;
        this.phase = phase;
        this.level = level;
        this.subsystem = subsystem;
        this.event = event;
        this.fields = Collections.unmodifiableMap(new LinkedHashMap<>(fields));
    }

    public long timestampMs() {
        return timestampMs;
    }

    public double matchTimeSec() {
        return matchTimeSec;
    }

    public String runId() {
        return runId;
    }

    public String opMode() {
        return opMode;
    }

    public String phase() {
        return phase;
    }

    public DecodeLogger.Level level() {
        return level;
    }

    public String subsystem() {
        return subsystem;
    }

    public String event() {
        return event;
    }

    public Map<String, Object> fields() {
        return fields;
    }

    public String fieldsAsString() {
        if (fields.isEmpty()) return "{}";

        StringJoiner joiner = new StringJoiner(", ", "{", "}");
        for (Map.Entry<String, Object> entry : fields.entrySet()) {
            joiner.add(entry.getKey() + "=" + String.valueOf(entry.getValue()));
        }
        return joiner.toString();
    }

    public String toCsvRow() {
        return timestampMs +
                "," + String.format(Locale.US, "%.3f", matchTimeSec) +
                "," + escape(runId) +
                "," + escape(opMode) +
                "," + escape(phase) +
                "," + escape(level.name()) +
                "," + escape(subsystem) +
                "," + escape(event) +
                "," + escape(fieldsAsString());
    }

    private static String escape(String value) {
        String text = value == null ? "" : value;
        return "\"" + text.replace("\"", "\"\"") + "\"";
    }
}
