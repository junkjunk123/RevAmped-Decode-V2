package org.firstinspires.ftc.teamcode.utils.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.logging.DecodeLogger;

public interface HwDevice {
    static <T> T init(HardwareMap hardwareMap, Class<T> c, String id) {
        try {
            return hardwareMap.get(c, id);
        } catch (Exception e) {
            DecodeLogger.get().error("hw", "HW_INIT_FAIL",
                    "deviceType", c.getSimpleName(),
                    "id", id,
                    "exception", e.getMessage());
            throw new IllegalArgumentException("motor " + id + " not found");
        }
    }

    default void update() {}
}
