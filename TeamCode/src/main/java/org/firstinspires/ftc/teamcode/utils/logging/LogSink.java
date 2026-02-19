package org.firstinspires.ftc.teamcode.utils.logging;

import java.util.Map;

public interface LogSink {
    String name();
    DecodeLogger.Level minLevel();

    void write(LogEvent event) throws Exception;

    default void snapshot(String tag, Map<String, Object> data) throws Exception {}

    default void flush(boolean force) throws Exception {}

    default void close() throws Exception {}
}
