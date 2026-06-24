package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.opmodes.OpModeCommand;

@TeleOp
@Disabled
public class FlywheelMotorTest extends OpModeCommand {
    private Flywheel flywheel;
    @Override
    public void initialize() {
        flywheel = new Flywheel(hardwareMap);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    @Override
    public void execute(){
        schedule(
            new Infinite(() -> {
                flywheel.update();
                telemetry.update();
            }),
            new Sequential(
                new Instant(() -> {
                    telemetry.addData("running", "right");
                }),
                testMotors(1.0,0),

                new Instant(() -> {
                    telemetry.clear();
                    telemetry.addData("running", "left");
                }),
                testMotors(1.0,1),

                new Instant(() -> {
                    telemetry.clear();
                    telemetry.addData("running","both");
                }),
                testMotors(1.0,0,1),
                new Instant(() -> {
                    telemetry.clear();
                    telemetry.addData("done","");
                })
            )
        );
    }

    public void runMotors(double power, int... index){
        for (int i = 0; i< index.length; i++){
            flywheel.hardware[i].setPower(power);
        }
    }

    public ICommand testMotors(double power, int...index){
        return new Sequential(
            new Instant(() -> runMotors(power,index)),
            new Wait(2000),
            new Instant(() -> runMotors(0,index)),
            new Wait(1000)
        );
    }
}
