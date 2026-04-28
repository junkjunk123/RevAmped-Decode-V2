package org.firstinspires.ftc.teamcode.utils.commands;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;

public class ServoProfileCommand extends Command {
    private final HwServo table;
    private double commandedPos;
    private final double endPos;
    private final double angularSpeed;
    private double error;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime totalTime = new ElapsedTime();

    public ServoProfileCommand(Table servo, double startPos, double endPos, double normalizedSpeed) {
        table = servo;
        this.commandedPos = startPos;
        this.endPos = endPos;
        this.angularSpeed = Math.abs(normalizedSpeed * (Math.abs(Table.BALL1 - Table.BALL1_END) / Table.AUTO_FAST_SHOOT_DELAY)); // tick/ms
        error = endPos - startPos;
    }

    @Override
    public void start() {
        timer.reset();
        totalTime.reset();
    }

    @Override
    public void execute() {
        double dt = timer.milliseconds();
        timer.reset();
        error = endPos - commandedPos;
        double step = Math.signum(error) * Math.min(Math.abs(error), angularSpeed * dt);
        commandedPos += step;
        commandedPos = Range.clip(commandedPos, 0.0, 1.0);
        table.setPosition(commandedPos);
    }

    @Override
    public boolean done() {
        return Math.abs(error) <= 0.01;
    }
}
