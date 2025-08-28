package org.firstinspires.ftc.teamcode.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config       //if you want config
@TeleOp(name="Dashboard Speed Control", group="Testing")

//@TeleOp       //if this is a teleop
//@Autonomous   //if this is an auto
public class TestOp extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public static double DRIVE_POWER = 0.5;  // default speed (0.0 - 1.0)

    GamepadEx g1;

    //this section allows us to access telemetry data from a browser
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    DcMotor myMotor;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step
        g1 = new GamepadEx(gamepad1);

        myMotor = hardwareMap.get(DcMotor.class,"myMotor");
        // Tell the driver that initialization is complete.
        dashboardTelemetry.addData("Status", "Initialized");
        dashboardTelemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
        myMotor.setPower(0.5);

    }
    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        g1.readButtons();

        // ✅ Use DRIVE_POWER to control the motor
        myMotor.setPower(DRIVE_POWER);

        // Send info to Dashboard telemetry
        dashboardTelemetry.addData("Status", "Run Time: " + runtime.toString());
        dashboardTelemetry.addData("Drive Power", DRIVE_POWER);
        dashboardTelemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}