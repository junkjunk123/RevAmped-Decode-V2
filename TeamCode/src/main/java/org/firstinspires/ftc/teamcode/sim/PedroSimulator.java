package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PedroSimulator {
    public static double dt;
    private ElapsedTime timer = new ElapsedTime();
    private PedroRobot robot;

    public PedroSimulator() {
        robot = new PedroRobot();
    }

    public void update() {
        robot.update();
    }

    public double elapsedTime() {
        return timer.milliseconds();
    }

    public boolean isFinished() {
        return !robot.follower.isBusy();
    }

    public static void main(String[] args) {
        PedroSimulator simulator = new PedroSimulator();
        while (simulator.elapsedTime() < 2000 && !simulator.isFinished()) {
            simulator.update();
        }
    }
}
