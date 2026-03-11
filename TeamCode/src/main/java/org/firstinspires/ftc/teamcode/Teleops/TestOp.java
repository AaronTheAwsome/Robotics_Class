package org.firstinspires.ftc.teamcode.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;
import java.sql.Driver;

@Config
@TeleOp(name = "BattleDroid Manual Control")
public class TestOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public static double DRIVE_POWER = 0;
    public static double LAUNCH_POWER = 0;
    public static int position = 10000;

    private GamepadEx g1;

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();

    // subsystems
    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;

    @Override
    public void init() {

        g1 = new GamepadEx(gamepad1);

        drive   = new DriveSubsystem(hardwareMap);
        intake  = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);

        runtime.reset();


        dashboardTelemetry.addData("Status", "Initialized");
        dashboardTelemetry.addData("launchSpeed","launchMotor");
        dashboardTelemetry.update();
    }

    @Override
    public void start() {
        intake.setStartPositions();
    }

    @Override
    public void loop() {
        g1.readButtons();

        double driveInput = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double turnInput = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        drive.drive2( driveInput, turnInput, rx);

        // heading hold (your previous logic)

        // intake / lift
        if (gamepad1.x) {
            intake.intakeAndLift();
        } else if (gamepad1.circle) {
            intake.stopIntakeAndLift();
        }

        if (gamepad1.triangle) {
            intake.dumpPosition();
        } else {
            intake.neutralPosition();
        }

        if (g1.wasJustPressed(com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER)) {
            intake.liftUp();
        }
        if (g1.wasJustReleased(com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER)) {
            intake.liftStop();
        }

        // shooter
        if (g1.wasJustPressed(com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER)) {
            shooter.shoot();
        }
        //if (g1.wasJustReleased(com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER)) {
        //    shooter.stop();
       // }

        // Telemetry

        dashboardTelemetry.addData("position", position);
        dashboardTelemetry.addData("Launch Power", LAUNCH_POWER);
        dashboardTelemetry.addData("Drive Power", DRIVE_POWER);
        dashboardTelemetry.addData("lift ticks", intake.getLiftTicks());
        dashboardTelemetry.addData("Status", "Run Time: " + runtime.toString());
        dashboardTelemetry.update();
    }

    @Override
    public void stop() {
        intake.stop();
        shooter.stop();
    }
}
