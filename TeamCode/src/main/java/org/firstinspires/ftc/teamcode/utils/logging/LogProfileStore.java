package org.firstinspires.ftc.teamcode.utils.logging;

import android.content.Context;
import android.content.SharedPreferences;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

public final class LogProfileStore {
    private static final String PREFS_NAME = "decode_logging";
    private static final String KEY_PROFILE = "profile";
    private static final LogProfile DEFAULT_PROFILE = LogProfile.PRACTICE;

    private LogProfileStore() {}

    public static LogProfile load() {
        try {
            SharedPreferences preferences = preferences();
            String raw = preferences.getString(KEY_PROFILE, DEFAULT_PROFILE.name());
            return LogProfile.fromName(raw, DEFAULT_PROFILE);
        } catch (Exception ignored) {
            return DEFAULT_PROFILE;
        }
    }

    public static boolean save(LogProfile profile) {
        if (profile == null) return false;
        try {
            return preferences()
                    .edit()
                    .putString(KEY_PROFILE, profile.name())
                    .commit();
        } catch (Exception ignored) {
            return false;
        }
    }

    private static SharedPreferences preferences() {
        Context context = AppUtil.getDefContext();
        return context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
    }
}
