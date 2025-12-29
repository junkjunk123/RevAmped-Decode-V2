package org.firstinspires.ftc.teamcode.utils.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface HwDevice {
    static <T> T init(HardwareMap hardwareMap, Class<T> c, String id) {
        try {
            return hardwareMap.get(c, id);
        } catch (Exception e) {
            throw new IllegalArgumentException("motor " + id + " not found");
        }
    }

    default void update() {}
}
