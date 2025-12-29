package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.GamepadEx;
import org.firstinspires.ftc.teamcode.utils.prompter.OptionPrompt;
import org.firstinspires.ftc.teamcode.utils.prompter.Prompter;

import java.util.HashMap;
import java.util.List;

@Config
@TeleOp(name = "ServoCalibrateTest", group = "Test")
public class ServoCalibrateTest extends OpMode {
    private ServoImplEx servo;
    private int posJoy1;
    private long timeStamp = 0;
    public static int denominator = 255;
    public static int TICK_CHANGE = 1;
    private Telemetry telemetryA;
    private boolean singleTickMode = false;
    private int selectedPort = 0;
    private List<String> servos;
    private String currentServo;
    //private AnalogInput voltage;
    private final HashMap<String, Integer> calibratedPositions = new HashMap<>();
    private Prompter prompter;
    private GamepadEx gamepad_1;

    private enum TestState {
        SELECT,
        CALIBRATE
    }
    private TestState testState = TestState.SELECT;

    @Override
    public void init() {
        gamepad_1 = new GamepadEx(gamepad1);
        servos = List.of("hood","popper","table");
        telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //voltage = hardwareMap.get(AnalogInput.class,"popper_voltage");
        telemetry.addData(">>", "Press start to continue");
        telemetry.update();
        prompter = new Prompter(this, gamepad_1)
                .prompt("servo", new OptionPrompt<>(
                        "Select a servo -- press right bumper to select",
                        servos.toArray(new String[0])
                ))
                .onComplete(() -> {currentServo = prompter.get("servo"); testState = TestState.CALIBRATE;})
                .thenDisplay(() -> "Selected servo: " + currentServo);
    }

    @Override
    public void init_loop() {
        prompter.run();
    }

    @Override
    public void start() {
        selectedPort = servos.indexOf(currentServo);
        servo = hardwareMap.get(ServoImplEx.class, currentServo);
        int initialPos = (int) servo.getPosition() * 255;
        if (initialPos == 0) {
            posJoy1 = denominator / 2;
        } else {
            posJoy1 = initialPos;
        }
        calibratedPositions.put(currentServo, posJoy1);
    }

    @Override
    public void loop() {
        switch (testState) {
            case CALIBRATE -> {
                gamepad_1.update();
                timeStamp = System.currentTimeMillis();
                if (calibratedPositions.containsKey(currentServo))
                    calibratedPositions.replace(currentServo, posJoy1);
                if (!singleTickMode) {
                    if (gamepad_1.x.canPress4Short(timeStamp)) {
                        posJoy1 -= TICK_CHANGE;
                    } else if (gamepad_1.b.canPress4Short(timeStamp)) {
                        posJoy1 += TICK_CHANGE;
                    }
                } else {
                    if (gamepad_1.x.canPress(timeStamp)) {
                        posJoy1 -= TICK_CHANGE;
                    } else if (gamepad_1.b.canPress(timeStamp)) {
                        posJoy1 += TICK_CHANGE;
                    }
                }

                if (gamepad_1.y.canPress(timeStamp)) {
                    singleTickMode = !singleTickMode;
                } else if (gamepad_1.right_stick_button.canPress(timeStamp)) {
                    posJoy1 = 128;
                }

                posJoy1 = Range.clip(posJoy1, 0, denominator);
                servo.setPosition((float) posJoy1 / denominator);

                if (gamepad_1.right_bumper.isRisingEdge()) {
                    TICK_CHANGE += 5;
                    TICK_CHANGE = Range.clip(TICK_CHANGE, 1, 100);
                } else if (gamepad_1.left_bumper.isRisingEdge()) {
                    TICK_CHANGE -= 5;
                    TICK_CHANGE = Range.clip(TICK_CHANGE, 1, 100);
                } else if (gamepad_1.right_stick_button.isRisingEdge()) {
                    TICK_CHANGE = 1;
                }

                if (gamepad_1.back.isRisingEdge() || gamepad_1.a.isRisingEdge())
                    setTestState(TestState.SELECT);

                telemetryA.addData("Servo", currentServo);
                calibratedPositions.forEach((u, v) -> telemetryA.addData(u, Integer.toString(v)));
                telemetryA.addData("singleTickMode", singleTickMode);
                telemetryA.addData("TICK_CHANGE", TICK_CHANGE);
                telemetry.addData("PWM",
                        Integer.toString((int) (posJoy1 * 1805.2 / 255f + 591.68)));
                telemetryA.update();
            }
            case SELECT -> prompter.run();
        }
        //telemetryA.addData("Voltage",voltage.getVoltage());
    }

    private void setTestState(TestState state) {
        this.testState = state;

        switch (state) {
            case CALIBRATE -> {
                selectedPort = servos.indexOf(currentServo);
                servo = hardwareMap.get(ServoImplEx.class, currentServo);

                if (calibratedPositions.containsKey(currentServo)) {
                    posJoy1 = calibratedPositions.get(currentServo);
                } else {
                    int initialPos = (int) servo.getPosition() * 255;
                    if (initialPos == 0) {
                        posJoy1 = denominator / 2;
                    } else {
                        posJoy1 = initialPos;
                    }
                    calibratedPositions.put(currentServo, posJoy1);
                }
            }
            case SELECT -> prompter = new Prompter(this, gamepad_1)
                    .prompt("servo", new OptionPrompt<>(
                            "Select a servo -- press right bumper to select",
                            servos.toArray(new String[0])
                    ))
                    .onComplete(() -> {
                        currentServo = prompter.get("servo");
                        setTestState(TestState.CALIBRATE);
                    });
        }
    }
}
