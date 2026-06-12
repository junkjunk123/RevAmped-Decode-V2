package org.firstinspires.ftc.teamcode.sim;

import java.util.ArrayList;
import java.util.List;

public class MotorTrajectorySim {
    public static double kS = 0.1;
    public static double kV = 0.015;
    public static double kA = 0.001;
    public static double kU = 0.15;
    public static double x0 = 0;
    public static double v0 = 20;
    public static double DURATION = 1.0;
    private MotorModel model;
    private ArrayList<Double> times = new ArrayList<>();
    private ArrayList<Double> position = new ArrayList<>();
    private ArrayList<Double> velocity = new ArrayList<>();

    public MotorTrajectorySim() {
        model = new MotorModel(kS, kV, kA, kU, v0, x0);
    }

    public void run(double[] times, double[] controlInput) {
        model.reset();
        this.times.clear();
        position.clear();
        velocity.clear();

        model.update(times[0], controlInput[0]);
        record();

        while (model.getElapsedTime() <= DURATION && model.getIterations() < times.length) {
            int iteration = model.getIterations();
            model.update(times[iteration] - times[iteration - 1], controlInput[iteration]);
            record();
        }
    }

    private void record() {
        times.add(model.getElapsedTime());
        velocity.add(model.getState()[1]);
        position.add(model.getState()[0]);
    }

    public List<Double> times() {
        return times;
    }

    public List<Double> positions() {
        return position;
    }

    public List<Double> velocities() {
        return velocity;
    }
}
