package org.firstinspires.ftc.teamcode.utils.logging;

public enum LogProfile {
    MATCH(
            DecodeLogger.Level.ERROR,
            250,
            false,
            DecodeLogger.Level.ERROR,
            250,
            true,
            DecodeLogger.Level.ERROR,
            0
    ),
    PRACTICE(
            DecodeLogger.Level.INFO,
            100,
            true,
            DecodeLogger.Level.INFO,
            100,
            true,
            DecodeLogger.Level.INFO,
            10
    ),
    DEBUG(
            DecodeLogger.Level.DEBUG,
            75,
            true,
            DecodeLogger.Level.DEBUG,
            50,
            true,
            DecodeLogger.Level.DEBUG,
            1
    );

    private final DecodeLogger.Level telemetryMinLevel;
    private final int telemetryUpdateMs;
    private final boolean dashboardEnabled;
    private final DecodeLogger.Level dashboardMinLevel;
    private final int dashboardUpdateMs;
    private final boolean fileEnabled;
    private final DecodeLogger.Level fileMinLevel;
    private final int loopMsSampleEveryLoops;

    LogProfile(
            DecodeLogger.Level telemetryMinLevel,
            int telemetryUpdateMs,
            boolean dashboardEnabled,
            DecodeLogger.Level dashboardMinLevel,
            int dashboardUpdateMs,
            boolean fileEnabled,
            DecodeLogger.Level fileMinLevel,
            int loopMsSampleEveryLoops
    ) {
        this.telemetryMinLevel = telemetryMinLevel;
        this.telemetryUpdateMs = telemetryUpdateMs;
        this.dashboardEnabled = dashboardEnabled;
        this.dashboardMinLevel = dashboardMinLevel;
        this.dashboardUpdateMs = dashboardUpdateMs;
        this.fileEnabled = fileEnabled;
        this.fileMinLevel = fileMinLevel;
        this.loopMsSampleEveryLoops = loopMsSampleEveryLoops;
    }

    public DecodeLogger.Level telemetryMinLevel() {
        return telemetryMinLevel;
    }

    public int telemetryUpdateMs() {
        return telemetryUpdateMs;
    }

    public boolean dashboardEnabled() {
        return dashboardEnabled;
    }

    public DecodeLogger.Level dashboardMinLevel() {
        return dashboardMinLevel;
    }

    public int dashboardUpdateMs() {
        return dashboardUpdateMs;
    }

    public boolean fileEnabled() {
        return fileEnabled;
    }

    public DecodeLogger.Level fileMinLevel() {
        return fileMinLevel;
    }

    public int loopMsSampleEveryLoops() {
        return loopMsSampleEveryLoops;
    }

    public static LogProfile fromName(String raw, LogProfile fallback) {
        if (raw == null) return fallback;
        try {
            return LogProfile.valueOf(raw.trim().toUpperCase());
        } catch (IllegalArgumentException ignored) {
            return fallback;
        }
    }
}
