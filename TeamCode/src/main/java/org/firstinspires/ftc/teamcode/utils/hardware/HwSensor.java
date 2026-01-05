package org.firstinspires.ftc.teamcode.utils.hardware;
import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class HwSensor<T, S> implements HwDevice {
    private final String id;
    protected T cachedValue;
    protected S sensor;

    /**
     * Constructor for CachedSensor.
     * @param id the unique identifier for the sensor
     */
    public HwSensor(HardwareMap hardwareMap, String id, Class<S> targetClass) {
        sensor = HwDevice.init(hardwareMap, targetClass, id);
        this.id = id;
    }

    public T getReading() {
        return cachedValue != null ? cachedValue : (cachedValue = get());
    }

    public String getId() {
        return id;
    };

    @NonNull
    @Override
    public String toString() {
        return getId();
    }

    @Override
    public void update() {
        cachedValue = null;
    }

    public abstract T get();
}
