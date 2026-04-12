package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.SpindexerColorSensors;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.SpindexerCompartmentManager;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.utils.commands.GamepadEx;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.RandomizationState;
import org.firstinspires.ftc.teamcode.utils.prompter.OptionPrompt;
import org.firstinspires.ftc.teamcode.utils.prompter.Prompter;

@TeleOp
public class SortingTest extends OpModeCommand {
    SpindexerColorSensors colorSensors;
    SpindexerCompartmentManager compartmentManager;
    GamepadEx gamepad_1;
    Prompter prompter;
    MultipleTelemetry telemetries;
    Robot robot;

    @Override
    public void initialize() {
        Globals.randomizationState = RandomizationState.PPG;
        gamepad_1 = new GamepadEx(gamepad1);
        robot = new Robot(hardwareMap);
        prompter = new Prompter(this,gamepad_1);
        //colorSensors = new SpindexerColorSensors(hardwareMap,"colorLeft","colorRight");
        compartmentManager = new SpindexerCompartmentManager(colorSensors);
        telemetries = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        prompter.prompt("motif", new OptionPrompt<>("Select a motif -- press right bumper to select", RandomizationState.values()))
                .onComplete(() -> {
                    Globals.randomizationState = prompter.get("motif");
                })
                .thenDisplay(() -> "Selected motif: " + Globals.randomizationState);

        schedule(new Infinite(() -> {
            robot.update();
            colorSensors.update();
            compartmentManager.update();
        }));

        schedule(new Sequential(
                new WaitUntil(() -> !opModeInInit()),
                new Instant(robot::initialize),
                new Instant(()->compartmentManager.updateMotifIndex())
                )
        );
    }

    @Override
    public void initializeLoop() {
        prompter.run();
    }

    @Override
    public void execute() {
        gamepad_1.update();

        if (gamepad_1.y.isRisingEdge()){
            int offset = compartmentManager.getOffset();
            if(offset>0) {
                schedule(robot.table.next());
            } else if(offset<0){
                schedule(robot.table.previous());
            }
        }
        if (gamepad_1.dpad_up.isRisingEdge()){
            schedule(new Sequential(
                new Parallel(
                    new Instant(() -> robot.flywheel.near()),
                    robot.popper.pop()
                ),
                new Wait(500),
                robot.shootAll(),
                robot.resetAfterShooting()
            ));
        }
        telemetries.addData("Colors",colorSensors.getCompartmentColors());
        telemetries.addData("Offset",compartmentManager.getOffset());
        telemetries.addData("Green Index",compartmentManager.getGreenIndex());
        telemetries.addData("Motif Green Index",Globals.randomizationState.getGreenIndex());
        telemetries.update();
    }
}
