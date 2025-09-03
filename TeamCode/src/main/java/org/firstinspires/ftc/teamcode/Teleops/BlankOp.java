package org.firstinspires.ftc.teamcode.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name="MyMotorWithEncoder")
public class BlankOp extends OpMode {
    // Timer
    private ElapsedTime runtime = new ElapsedTime();

    // Configurable motor power
    public static double DRIVE_POWER = 0.5;

    // Encoder target positions (you can tune in FTC Dashboard)
    public static int targetUpPosition = 1000;
    public static int targetDownPosition = 0;

    // Gamepad helper
    GamepadEx g1;

    // FTC Dashboard
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    // Motor
    DcMotor myMotor;

    @Override
    public void init() {
        g1 = new GamepadEx(gamepad1);

        // Initialize motor
        myMotor = hardwareMap.get(DcMotor.class,"myMotor");

        // Reset encoder and set to run-to-position
        myMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor.setTargetPosition(targetDownPosition);
        myMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        dashboardTelemetry.addData("Status", "Initialized");
        dashboardTelemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
        myMotor.setPower(DRIVE_POWER);
    }

    @Override
    public void loop() {
        g1.readButtons();

        // Button controls for encoder positions
        if (gamepad1.a) {
            myMotor.setTargetPosition(targetUpPosition);
            myMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myMotor.setPower(DRIVE_POWER);
        }
        if (gamepad1.b) {
            myMotor.setTargetPosition(targetDownPosition);
            myMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myMotor.setPower(DRIVE_POWER);
        }

        // Telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Target Position", myMotor.getTargetPosition());
        telemetry.addData("Current Position", myMotor.getCurrentPosition());
        telemetry.addData("Drive Power", DRIVE_POWER);
        telemetry.update();

        dashboardTelemetry.addData("Status", "Run Time: " + runtime.toString());
        dashboardTelemetry.addData("Target Position", myMotor.getTargetPosition());
        dashboardTelemetry.addData("Current Position", myMotor.getCurrentPosition());
        dashboardTelemetry.addData("Drive Power", DRIVE_POWER);
        dashboardTelemetry.update();
    }

    @Override
    public void stop() {
        myMotor.setPower(0);
    }
}
