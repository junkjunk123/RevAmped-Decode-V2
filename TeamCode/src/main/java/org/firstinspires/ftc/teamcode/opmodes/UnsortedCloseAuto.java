package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Deadline;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Popper;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.opmodes.paths.UnsortedCloseAutoPaths;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;

public class UnsortedCloseAuto extends OpModeCommand {
    private Robot robot;
    private final ElapsedTime overallTimer = new ElapsedTime();
    private boolean transferredData = false;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, new UnsortedCloseAutoPaths());
        robot.turret.setPosition(ServoTurret.UNSORTED_AUTO_PRELOADS);
        robot.popper.popCommandless();
        robot.table.setStateCommandless(Table.RelativeState.BALL1);
        robot.hood.unsortedAuto();

        schedule(
                new Infinite(() -> {
                    robot.update();
                    Drivetrain.startPose = robot.drivetrain.follower.getPose();
                    telemetry.addData("turret target", robot.turret.getPosition());
                    telemetry.addData("turret encoder", robot.turret.getPosition());
                    telemetry.update();
                }),
                new Sequential(
                        new WaitUntil(() -> !opModeInInit()),
                        new Instant(overallTimer::reset),
                        new Instant(() -> robot.flywheel.setVelocity(Flywheel.UNSORTED_AUTO_VELOCITY + 60)),
                        new Deadline(
                                robot.drivetrain.followNext(d -> d.tValueCondition(0.9), 3000),
                                new Sequential(
                                        new WaitUntil(() -> robot.drivetrain.tValueCondition(0.4)),
                                        new Parallel(
                                                robot.autoFastShoot(),
                                                new Sequential(
                                                        new Wait(50),
                                                        new Instant(() -> {robot.turret.setPosition(
                                                                robot.turret.getPosition() -
                                                                        20/255f * (int) Math.signum(robot.turret.getPosition())
                                                        ); robot.flywheel.setVelocity(Flywheel.UNSORTED_AUTO_VELOCITY + 115);})
                                                )
                                        )
                                )
                        ),
                        intakePreload(true),
                        shootPreload(true),
                        intakeFromGate(0),
                        shootFromGate(0),
                        intakeFromGate(1),
                        shootFromGate(1),
                        intakeFromGate(2),
                        shootFromGate(2),
                        //intakeFromGate(3),
                        //shootFromGate(3),
                        intakePreload(false),
                        shootPreload(false),
                        park()
                )
        );
    }

    private ICommand intakeFromGate(int iteration) {
        return AutoMethods.intakeFromGate(robot, iteration);
    }

    private ICommand shootFromGate(int iteration) {
        return AutoMethods.shootFromGate(robot, iteration, false);
    }

    private ICommand intakePreload(boolean isFirst) {
        return AutoMethods.intakePreload(robot, isFirst);
    }

    private ICommand shootPreload(boolean isFirst) {
        return AutoMethods.shootPreload(robot, isFirst);
    }

    @Override
    public void end() {
        if (!transferredData) Globals.turretStartPos = robot.turret.getPosition();
        //Drivetrain.startPose = robot.drivetrain.follower.getPose();
    }

    public ICommand park() {
        return new Parallel(
                robot.drivetrain.followNext(d -> d.velocityCondition(4)),
                new Instant(() -> {
                    robot.flywheel.stop();
                    robot.intakeMotor.stop();
                    Globals.turretStartPos = robot.turret.getPosition();
                    transferredData = true;
                }),
                new Conditional(
                        () -> robot.popper.atState(Popper.PopperState.NEUTRAL),
                        robot.table.reset(),
                        new Sequential(
                                robot.popper.neutral(),
                                robot.table.reset()
                        )
                )
        );
    }

    @Override
    public void execute() {
        telemetry.update();
    }

    @Override
    public void initializeLoop() {
        robot.update();
    }
}
