package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.ivy.commands.Infinite;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.utils.commands.GamepadEx;

@TeleOp
@Disabled
public class defenseTele extends OpModeCommand {
    private Drivetrain drivetrain;
    private GamepadEx gamepad_1;
    @Override
    public void initialize() {
        drivetrain = new Drivetrain(hardwareMap);
        gamepad_1 = new GamepadEx(gamepad1);
        schedule(
                new Infinite(()-> {
                    drivetrain.update();
                    drivetrain.arcadeDrive(gamepad1);
                })
        );
    }

    @Override
    public void execute(){
        gamepad_1.update();
    }
}
