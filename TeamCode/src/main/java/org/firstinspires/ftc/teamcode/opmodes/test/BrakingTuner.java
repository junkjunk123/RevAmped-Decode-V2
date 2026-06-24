package org.firstinspires.ftc.teamcode.opmodes.test;
import static org.firstinspires.ftc.teamcode.pedro.Tuning.follower;
import static org.firstinspires.ftc.teamcode.pedro.Tuning.stopRobot;

import android.annotation.SuppressLint;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Disabled
public class BrakingTuner extends OpMode {
    private static final double TEST_POWER = 1.0;
    private static final double BRAKING_POWER = 0;

    private static final int DRIVE_TIME_MS = 1000;

    private enum State {
        START_MOVE,
        WAIT_DRIVE_TIME,
        APPLY_BRAKE,
        WAIT_BRAKE_TIME,
        RECORD,
        DONE
    }

    private static class BrakeRecord {
        double timeMs;
        Pose pose;
        double velocity;

        BrakeRecord(double timeMs, Pose pose, double velocity) {
            this.timeMs = timeMs;
            this.pose = pose;
            this.velocity = velocity;
        }
    }

    private State state = State.START_MOVE;

    private final ElapsedTime timer = new ElapsedTime();

    private int iteration = 0;

    private Vector startPosition;
    private double measuredVelocity;
    private double distance;
    private TelemetryManager telemetryM;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void init_loop() {
        telemetryM.debug("The robot will move forwards and backwards starting at max speed and slowing down.");
        telemetryM.debug("Make sure you have enough room. Leave at least 4-5 feet.");
        telemetryM.debug("After stopping, kFriction and kBraking will be displayed.");
        telemetryM.debug("Make sure to turn the timer off.");
        telemetryM.debug("Press B on game pad 1 to stop.");
        telemetryM.update(telemetry);
        follower.update();
    }

    @Override
    public void start() {
        timer.reset();
        follower.update();
        follower.startTeleOpDrive(true);
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        follower.update();

        if (gamepad1.b) {
            stopRobot();
            requestOpModeStop();
            return;
        }

        double direction = (iteration % 2 == 0) ? 1 : -1;

        switch (state) {
            case START_MOVE: {
                follower.setMaxPower(TEST_POWER);
                follower.setTeleOpDrive(direction, 0, 0, true);

                timer.reset();
                state = State.WAIT_DRIVE_TIME;
                break;
            }

            case WAIT_DRIVE_TIME: {
                if (timer.milliseconds() >= DRIVE_TIME_MS) {
                    measuredVelocity = follower.getVelocity().getMagnitude();
                    startPosition = follower.getPose().getAsVector();
                    state = State.APPLY_BRAKE;
                }
                break;
            }

            case APPLY_BRAKE: {
                follower.setTeleOpDrive(BRAKING_POWER * direction, 0, 0, true);

                timer.reset();
                state = State.WAIT_BRAKE_TIME;
                break;
            }

            case WAIT_BRAKE_TIME: {
                if (follower.getVelocity().dot(new Vector(direction,
                        follower.getHeading())) <= 0) {
                    state = State.RECORD;
                }
                break;
            }

            case RECORD: {
                Vector endPosition = follower.getPose().getAsVector();
                distance = endPosition.minus(startPosition).getMagnitude();

                telemetryM.debug("Test " + iteration,
                        String.format("v=%.3f  d=%.3f", measuredVelocity,
                                distance));
                telemetryM.update(telemetry);
                state = State.DONE;

                break;
            }

            case DONE: {
                stopRobot();
                telemetryM.debug("measuredVelocity", measuredVelocity);
                telemetryM.debug("distance", distance);
                telemetryM.update();
                break;
            }
        }
    }
}