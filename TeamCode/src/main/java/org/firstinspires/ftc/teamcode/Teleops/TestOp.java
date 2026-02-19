package org.firstinspires.ftc.teamcode.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Config
@TeleOp(name = "Filthy Clanker Code")
public class TestOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    GamepadEx g1;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    // subsystems
    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;

    @Override
    public void init() {
        g1 = new GamepadEx(gamepad1);

        drive = new DriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);

        runtime.reset();
        dashboardTelemetry.addData("Status", "Initialized");
        dashboardTelemetry.update();
    }

    @Override
    public void start() {
        // shooter constructor already set servos to neutral
    }

    @Override
    public void loop() {
        g1.readButtons();

        double driveInput = -gamepad1.left_stick_y;  // usually inverted
        double turnInput  = gamepad1.right_stick_x;

        // driving
        drive.drive(driveInput, turnInput);

        // intake + slow spin-up when X pressed
        if (gamepad1.x) {
            intake.intakeIn();
            shooter.spinUp(0.2);
        } else if (gamepad1.circle) {
            intake.stop();
            shooter.spinOff();
        }

        // feeder servos with triangle
        if (gamepad1.triangle) {
            shooter.feed();
        } else {
            shooter.neutral();
        }

        // full power flywheel on left bumper
        if (g1.wasJustPressed(GamepadEx.GamepadButton.LEFT_BUMPER)) {
            shooter.spinUp(1.0);
        } else if (g1.wasJustReleased(GamepadEx.GamepadButton.LEFT_BUMPER)) {
            shooter.spinOff();
        }

        // elevator on right bumper
        if (g1.wasJustPressed(GamepadEx.GamepadButton.RIGHT_BUMPER)) {
            shooter.elevatorOn();
        } else if (g1.wasJustReleased(GamepadEx.GamepadButton.RIGHT_BUMPER)) {
            shooter.elevatorOff();
        }

        // telemetry
        dashboardTelemetry.addData("yaw", drive.getYaw());
        dashboardTelemetry.addData("flywheel ticks", shooter.getFlywheelTicks());
        dashboardTelemetry.addData("intake ticks", intake.getTicks());
        dashboardTelemetry.addData("left drive ticks", drive.getLeftTicks());
        dashboardTelemetry.addData("right drive ticks", drive.getRightTicks());
        dashboardTelemetry.addData("Status", "Run Time: " + runtime.toString());
        dashboardTelemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
        intake.stop();
        shooter.spinOff();
        shooter.elevatorOff();
    }
}
