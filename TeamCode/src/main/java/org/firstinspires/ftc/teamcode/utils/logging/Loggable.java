package org.firstinspires.ftc.teamcode.utils.logging;

import java.util.Map;

public interface Loggable {
    String logTag();
    Map<String, Object> snapshot();
}
