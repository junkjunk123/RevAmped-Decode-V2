package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.utils.hardware.HwDevice;

import java.util.List;

public interface Types {
    enum TableState implements HwDevice {
        BALL1,
        BALL2,
        BALL3,
        BALL1_REV2,
        BALL2_REV2,
        BALL1_END,
        BALL2_END,
        BALL3_END;

        public float getPosition() {
            switch (this) {
                case BALL1 -> {
                    return TableServo.BALL1;
                }
                case BALL2 -> {
                    return TableServo.BALL2;
                }
                case BALL3 -> {
                    return TableServo.BALL3;
                }
                case BALL1_END -> {
                    return TableServo.BALL1_END;
                }
                case BALL2_END -> {
                    return TableServo.BALL2_END;
                }
                case BALL3_END -> {
                    return TableServo.BALL3_END;
                }
                case BALL1_REV2 -> {
                    return TableServo.BALL1 + fullRotation();
                }
                case BALL2_REV2 -> {
                    return TableServo.BALL2 + fullRotation();
                }
            }

            return TableServo.BALL1;
        }

        public static float fullRotation() {
            return (BALL2.getPosition() - BALL1.getPosition()) * 3f;
        }

        public static List<Table> relativeStates() {
            return List.of(BALL1, BALL2, BALL3);
        }
    }
}
