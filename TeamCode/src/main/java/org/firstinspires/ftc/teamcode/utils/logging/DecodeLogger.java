package org.firstinspires.ftc.teamcode.utils.logging;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

public final class DecodeLogger {
    public enum Level { DEBUG, INFO, WARN, ERROR }

    private static final int MAX_BUFFERED_EVENTS = 512;
    private static final int FLUSH_LOOP_INTERVAL = 10;
    private static final long SNAPSHOT_PERIOD_NANOS = 100_000_000L;

    private static volatile DecodeLogger instance = new DecodeLogger(true);

    private static final class SinkState {
        final LogSink sink;
        boolean failed;

        SinkState(LogSink sink) {
            this.sink = sink;
        }
    }

    private final boolean noOp;
    private final RunContext context;
    private final ArrayDeque<LogEvent> bufferedEvents = new ArrayDeque<>();
    private final List<SinkState> sinks = new ArrayList<>();
    private final Map<String, Long> snapshotTagTimes = new LinkedHashMap<>();
    private boolean closed;
    private boolean inFailover;
    private int loopsSinceFlush;
    private Level minEnabledLevel = Level.ERROR;

    private DecodeLogger(boolean noOp) {
        this.noOp = noOp;
        this.context = null;
    }

    private DecodeLogger(RunContext context) {
        this.noOp = false;
        this.context = context;
    }

    public static synchronized void init(Telemetry dsTelemetry, String opModeName, boolean enableFileSink) {
        init(dsTelemetry, opModeName, LogProfile.PRACTICE, enableFileSink);
    }

    public static synchronized void init(Telemetry dsTelemetry, String opModeName, LogProfile profile) {
        init(dsTelemetry, opModeName, profile, true);
    }

    private static synchronized void init(Telemetry dsTelemetry, String opModeName, LogProfile profile, boolean enableFileSink) {
        if (instance != null && !instance.noOp) {
            instance.close();
        }

        LogProfile activeProfile = profile == null ? LogProfile.PRACTICE : profile;
        RunContext runContext = new RunContext(opModeName);
        DecodeLogger logger = new DecodeLogger(runContext);

        if (dsTelemetry != null) {
            logger.addSink(new TelemetrySink(
                    dsTelemetry,
                    activeProfile.telemetryMinLevel(),
                    activeProfile.telemetryUpdateMs()
            ));
        }

        if (activeProfile.dashboardEnabled()) {
            try {
                logger.addSink(new DashboardSink(
                        activeProfile.dashboardMinLevel(),
                        activeProfile.dashboardUpdateMs()
                ));
            } catch (Exception ignored) {
                // Dashboard is optional during init.
            }
        }

        if (enableFileSink && activeProfile.fileEnabled()) {
            try {
                logger.addSink(new FileSink(runContext.getRunId(), activeProfile.fileMinLevel()));
            } catch (Exception ignored) {
                // File sink is optional during init.
            }
        }

        instance = logger;
    }

    public static DecodeLogger get() {
        return instance == null ? new DecodeLogger(true) : instance;
    }

    public synchronized void setPhase(String phase) {
        if (noOp || closed) return;
        context.setPhase(phase);
    }

    public synchronized void markStart() {
        if (noOp || closed) return;
        context.markStart();
    }

    public synchronized String runId() {
        if (noOp || closed) return "noop";
        return context.getRunId();
    }

    public synchronized double matchTimeSec() {
        if (noOp || closed) return 0.0;
        return context.matchTimeSec(System.nanoTime());
    }

    public synchronized void event(Level level, String subsystem, String event, Map<String, Object> fields) {
        if (noOp || closed) return;
        if (level.ordinal() < minEnabledLevel.ordinal()) return;

        LogEvent logEvent = createEvent(level, subsystem, event, fields);
        if (level.ordinal() >= Level.WARN.ordinal()) {
            drainBufferedEvents();
            writeToSinks(logEvent);
            flushSinks(true, false);
            return;
        }

        enqueue(logEvent);
    }

    public void debug(String subsystem, String event, Object... kvPairs) {
        event(Level.DEBUG, subsystem, event, kvToMap(kvPairs));
    }

    public void info(String subsystem, String event, Object... kvPairs) {
        event(Level.INFO, subsystem, event, kvToMap(kvPairs));
    }

    public void warn(String subsystem, String event, Object... kvPairs) {
        event(Level.WARN, subsystem, event, kvToMap(kvPairs));
    }

    public void error(String subsystem, String event, Object... kvPairs) {
        event(Level.ERROR, subsystem, event, kvToMap(kvPairs));
    }

    public synchronized void snapshot(String tag, Map<String, Object> data) {
        if (noOp || closed || data == null || data.isEmpty()) return;

        long now = System.nanoTime();
        long last = snapshotTagTimes.getOrDefault(tag, 0L);
        if (now - last < SNAPSHOT_PERIOD_NANOS) return;
        snapshotTagTimes.put(tag, now);

        Map<String, Object> copy = new LinkedHashMap<>(data);
        for (SinkState state : sinks) {
            if (state.failed) continue;
            try {
                state.sink.snapshot(tag, copy);
            } catch (Exception e) {
                handleSinkFailure(state, e);
            }
        }
    }

    public synchronized void flush() {
        if (noOp || closed) return;

        drainBufferedEvents();
        loopsSinceFlush++;
        if (loopsSinceFlush >= FLUSH_LOOP_INTERVAL) {
            flushSinks(false, true);
            loopsSinceFlush = 0;
        }
    }

    public synchronized void forceFlush() {
        if (noOp || closed) return;
        drainBufferedEvents();
        flushSinks(true, true);
        loopsSinceFlush = 0;
    }

    public synchronized void close() {
        if (noOp || closed) return;
        forceFlush();
        for (SinkState state : sinks) {
            if (state.failed) continue;
            try {
                state.sink.close();
            } catch (Exception ignored) {
                // Ignore close failures at shutdown.
            }
        }
        closed = true;
    }

    private void addSink(LogSink sink) {
        sinks.add(new SinkState(sink));
        if (sink.minLevel().ordinal() < minEnabledLevel.ordinal()) {
            minEnabledLevel = sink.minLevel();
        }
    }

    private LogEvent createEvent(Level level, String subsystem, String event, Map<String, Object> fields) {
        long now = System.nanoTime();
        return new LogEvent(
                context.timestampMs(now),
                context.matchTimeSec(now),
                context.getRunId(),
                context.getOpModeName(),
                context.getPhase(),
                level,
                sanitizeSubsystem(subsystem),
                sanitizeEvent(event),
                sanitizeFields(fields)
        );
    }

    private void enqueue(LogEvent event) {
        if (bufferedEvents.size() >= MAX_BUFFERED_EVENTS) {
            if (!dropOldest(Level.DEBUG) && !dropOldest(Level.INFO)) {
                bufferedEvents.pollFirst();
            }
        }
        bufferedEvents.addLast(event);
    }

    private boolean dropOldest(Level level) {
        Iterator<LogEvent> it = bufferedEvents.iterator();
        while (it.hasNext()) {
            if (it.next().level() == level) {
                it.remove();
                return true;
            }
        }
        return false;
    }

    private void drainBufferedEvents() {
        while (!bufferedEvents.isEmpty()) {
            writeToSinks(bufferedEvents.pollFirst());
        }
    }

    private void flushSinks(boolean force, boolean includeTelemetrySink) {
        for (SinkState state : sinks) {
            if (state.failed) continue;
            if (!includeTelemetrySink && state.sink instanceof TelemetrySink) continue;
            try {
                state.sink.flush(force);
            } catch (Exception e) {
                handleSinkFailure(state, e);
            }
        }
    }

    private void writeToSinks(LogEvent event) {
        for (SinkState state : sinks) {
            if (state.failed) continue;
            if (event.level().ordinal() < state.sink.minLevel().ordinal()) continue;
            try {
                state.sink.write(event);
            } catch (Exception e) {
                handleSinkFailure(state, e);
            }
        }
    }

    private void handleSinkFailure(SinkState failedState, Exception exception) {
        if (failedState.failed) return;
        failedState.failed = true;
        if (inFailover) return;

        inFailover = true;
        try {
            LogEvent failoverEvent = createEvent(
                    Level.WARN,
                    "logging",
                    "SINK_FAILOVER",
                    kvToMap(
                            "sink", failedState.sink.name(),
                            "reason", exception == null ? "unknown" : exception.getClass().getSimpleName() + ": " + exception.getMessage()
                    )
            );
            for (SinkState state : sinks) {
                if (state.failed) continue;
                if (failoverEvent.level().ordinal() < state.sink.minLevel().ordinal()) continue;
                try {
                    state.sink.write(failoverEvent);
                    state.sink.flush(true);
                } catch (Exception ignored) {
                    state.failed = true;
                }
            }
        } finally {
            inFailover = false;
        }
    }

    private static Map<String, Object> sanitizeFields(Map<String, Object> fields) {
        Map<String, Object> out = new LinkedHashMap<>();
        if (fields == null) return out;
        for (Map.Entry<String, Object> entry : fields.entrySet()) {
            String key = entry.getKey() == null ? "null_key" : entry.getKey();
            out.put(key, entry.getValue());
        }
        return out;
    }

    private static String sanitizeSubsystem(String subsystem) {
        if (subsystem == null || subsystem.isBlank()) return "unknown";
        return subsystem.toLowerCase(Locale.US);
    }

    private static String sanitizeEvent(String event) {
        if (event == null || event.isBlank()) return "UNKNOWN_EVENT";
        return event.toUpperCase(Locale.US);
    }

    private static Map<String, Object> kvToMap(Object... kvPairs) {
        Map<String, Object> map = new LinkedHashMap<>();
        if (kvPairs == null) return map;
        for (int i = 0; i < kvPairs.length; i += 2) {
            Object key = kvPairs[i];
            Object value = i + 1 < kvPairs.length ? kvPairs[i + 1] : null;
            map.put(String.valueOf(key), value);
        }
        return map;
    }
}
