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
@TeleOp
public class BlankOp extends OpMode {
    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    public static double DRIVE_POWER = 0.5;  // default speed (0.0 - 1.0)

    GamepadEx g1;

    // Dashboard telemetry
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    DcMotor myMotor;

    @Override
    public void init() {
        g1 = new GamepadEx(gamepad1);

        myMotor = hardwareMap.get(DcMotor.class,"myMotor");

        // ✅ Setup encoder
        myMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        dashboardTelemetry.addData("Status", "Initialized");
        dashboardTelemetry.update();
    }

    @Override
    public void init_loop() { }

    @Override
    public void start() {
        runtime.reset();
        myMotor.setPower(DRIVE_POWER);
    }

    @Override
    public void loop() {
        g1.readButtons();

        // ✅ Control motor with power
        myMotor.setPower(DRIVE_POWER);

        // ✅ Encoder telemetry
        int encoderPosition = myMotor.getCurrentPosition();
        double velocity = myMotor.getPower();  // only works if motor supports it

        // Send info to Dashboard telemetry
        dashboardTelemetry.addData("Status", "Run Time: " + runtime.toString());
        dashboardTelemetry.addData("Drive Power", DRIVE_POWER);
        dashboardTelemetry.addData("Encoder Position", encoderPosition);
        dashboardTelemetry.addData("Velocity (ticks/sec)", velocity);
        dashboardTelemetry.update();
    }

    @Override
    public void stop() {
        myMotor.setPower(0);
        myMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // reset on stop if you want
    }
}
