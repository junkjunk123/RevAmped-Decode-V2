package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.octocanum.Octocanum;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;
import org.firstinspires.ftc.teamcode.utils.GamepadEx;

@TeleOp
public class drivetrainTele extends OpModeCommand {
    private Drivetrain drivetrain;
    private Octocanum octocanum;
    private GamepadEx gamepad_1;
    @Override
    public void initialize() {
        drivetrain = new Drivetrain(hardwareMap);
        octocanum = new Octocanum(hardwareMap);
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
        if(gamepad_1.a.isRisingEdge()){
            schedule(new Instant(() -> octocanum.toggle()));
        }
    }
}
