package org.firstinspires.ftc.teamcode.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Robot")
//@Disabled
public class TeleOp extends OpMode {

    private DriveSubsystem myDriveTrain;
    private Shooter myShooter;
    private GamepadEx g1;

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void init() {
        myDriveTrain = new DriveSubsystem(hardwareMap);
        myShooter   = new Shooter(hardwareMap);
        g1          = new GamepadEx(gamepad1);

        myDriveTrain.setHeadingToMaintain(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Target Heading (deg)", myDriveTrain.getHeadingToMaintain());
        telemetry.addData("Current Heading (deg)", myDriveTrain.getCurrentHeadingDeg());
        telemetry.update();
    }

    @Override
    public void start() {
        myDriveTrain.setHeadingToMaintain(myDriveTrain.getCurrentHeadingDeg());
    }

    @Override
    public void loop() {
        double y = 0.0;
        double x = 0.0;
        double rx = 0.0;

        g1.readButtons();
        if (-gamepad1.left_stick_y < 0.3) {
            y = -gamepad1.left_stick_y;
        }else {
            y = 0.0;
        }

        if (-gamepad1.left_stick_x < 0.3) {
            x = -gamepad1.left_stick_x;
        }else {
            x = 0.0;
        }

        if (-gamepad1.left_stick_x < 0.3) {
            rx = gamepad1.left_stick_x;
        }else {
            rx = 0.0;
        }

        if (g1.wasJustPressed(GamepadKeys.Button.B)) {
            myShooter.toggleMotor();
        }

        if (g1.wasJustPressed(GamepadKeys.Button.X)) {
            myShooter.servopos2();
        } else if (g1.wasJustReleased(GamepadKeys.Button.X)) {
            myShooter.servopos1();
        }

        myDriveTrain.drive2(x, y, rx);

        if (g1.wasJustPressed(GamepadKeys.Button.Y)) {
            myDriveTrain.setHeadingToMaintain(0);
        }

        telemetry.addData("Target Heading (deg)", myDriveTrain.getHeadingToMaintain());
        telemetry.addData("Current Heading (deg)", myDriveTrain.getCurrentHeadingDeg());
        telemetry.update();

        dashboardTelemetry.addData("Target Heading (deg)", myDriveTrain.getHeadingToMaintain());
        dashboardTelemetry.addData("Current Heading (deg)", myDriveTrain.getCurrentHeadingDeg());
        dashboardTelemetry.update();
    }

    @Override
    public void stop() {
        myDriveTrain.stopMotors();
    }
}
