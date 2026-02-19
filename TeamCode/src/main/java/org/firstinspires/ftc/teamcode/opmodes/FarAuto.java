package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.opmodes.paths.FarAutoPaths;
import org.firstinspires.ftc.teamcode.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.logging.DecodeLogger;

public class FarAuto extends OpModeCommand {
    private static final double TELEMETRY_PERIOD_MS = 100.0;
    private Robot robot;
    private ElapsedTime overallTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private String currentStep = "INIT";

    public FarAuto(AllianceColor allianceColor) {
        Globals.allianceColor = allianceColor;
    }

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, new FarAutoPaths());

        schedule(
                new Infinite(robot::update),
                new Sequential(
                        new Instant(overallTimer::reset),
                        step("PRELOAD_SETUP", new Instant(() -> {
                            robot.flywheel.far();
                            robot.hood.far();
                        })),
                        step("PRELOAD_ALIGN", new Parallel(
                                robot.popper.pop(),
                                robot.turret.runToPos(Turret.FAR_AUTO),
                                new Wait(750)
                        )),
                        step("PRELOAD_SHOOT", robot.shootAll(100)),
                        step("DRIVE_1", new Parallel(
                                robot.resetTable(),
                                new Instant(() -> robot.flywheel.stop()),
                                robot.drivetrain.followNext(d -> d.velocityCondition(4), 1500)
                        )),
                        step("DRIVE_2", robot.drivetrain.followNext(d -> d.velocityCondition(4), 2500)),
                        step("INTAKE_ENABLE", new Instant(robot.intakeMotor::intake)),
                        step("INTAKE_DELAY", new Wait(500)),
                        step("DRIVE_3", robot.drivetrain.followNext(d -> d.velocityCondition(4), 1500)),
                        step("FLYWHEEL_SPINUP", new Instant(() -> robot.flywheel.far())),
                        step("DRIVE_4", robot.drivetrain.followNext(d -> d.velocityCondition(4), 1500)),
                        step("FINAL_APPROACH", new Parallel(
                                robot.drivetrain.followNext(d -> d.velocityCondition(4), 2500),
                                new Sequential(
                                        new Wait(500),
                                        robot.popper.pop()
                                )
                        ))
                )
        );
    }

    @Override
    public void execute() {
        if (telemetryTimer.milliseconds() >= TELEMETRY_PERIOD_MS) {
            telemetry.addData("autoStep", currentStep);
            telemetry.addData("pathIndex", robot.drivetrain.getPathIndex());
            telemetry.addData("pathsRemaining", robot.drivetrain.getRemainingPaths());
            telemetry.update();
            telemetryTimer.reset();
        }
    }

    private ICommand step(String name, ICommand command) {
        return new Sequential(
                new Instant(() -> {
                    currentStep = name;
                    DecodeLogger.get().info("auto", "AUTO_STEP_START", "step", name);
                }),
                command,
                new Instant(() -> DecodeLogger.get().info("auto", "AUTO_STEP_COMPLETE",
                        "step", name,
                        "elapsedSec", overallTimer.seconds()))
        );
    }
}
