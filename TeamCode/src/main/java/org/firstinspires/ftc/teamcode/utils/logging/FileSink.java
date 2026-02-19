package org.firstinspires.ftc.teamcode.utils.logging;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
import java.util.Comparator;

public final class FileSink implements LogSink {
    private static final int MAX_FILES = 20;
    private static final String LOG_DIR = "/sdcard/FIRST/decode_logs";

    private final DecodeLogger.Level minLevel;
    private final File file;
    private final BufferedWriter writer;

    public FileSink(String runId, DecodeLogger.Level minLevel) throws IOException {
        this.minLevel = minLevel;
        File directory = new File(LOG_DIR);
        if (!directory.exists() && !directory.mkdirs()) {
            throw new IOException("Unable to create log directory: " + LOG_DIR);
        }

        rotate(directory);
        file = new File(directory, runId + ".csv");
        writer = new BufferedWriter(new FileWriter(file, false));
        writer.write("ts_ms,match_time_s,run_id,opmode,phase,level,subsystem,event,fields");
        writer.newLine();
    }

    @Override
    public String name() {
        return "FileSink(" + file.getAbsolutePath() + ")";
    }

    @Override
    public DecodeLogger.Level minLevel() {
        return minLevel;
    }

    @Override
    public synchronized void write(LogEvent event) throws IOException {
        writer.write(event.toCsvRow());
        writer.newLine();
    }

    @Override
    public synchronized void flush(boolean force) throws IOException {
        writer.flush();
    }

    @Override
    public synchronized void close() throws IOException {
        writer.flush();
        writer.close();
    }

    private static void rotate(File directory) {
        File[] files = directory.listFiles((dir, name) -> name.endsWith(".csv"));
        if (files == null || files.length < MAX_FILES - 1) return;

        Arrays.sort(files, Comparator.comparingLong(File::lastModified).reversed());
        for (int i = MAX_FILES - 1; i < files.length; i++) {
            files[i].delete();
        }
    }
}
