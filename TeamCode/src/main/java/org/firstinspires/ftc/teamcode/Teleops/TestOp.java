package org.firstinspires.ftc.teamcode.Teleops;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config       //if you want config
@TeleOp (name = "Aarons code")

//@TeleOp       //if this is a teleop
//@Autonomous   //if this is an auto
public class TestOp extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public static double DRIVE_POWER = 0.5;  // default speed (0.0 - 1.0)
    public static int positon = 10000;
    IMU imu;

    GamepadEx g1;
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
        imu = hardwareMap.get(IMU.class,"imu");
        myMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        myMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
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
        myMotor.setPower(DRIVE_POWER);

    }
    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        g1.readButtons();
        dashboardTelemetry.addData("yaw",imu.getRobotYawPitchRollAngles().getYaw());
        dashboardTelemetry.addData("pitch",imu.getRobotYawPitchRollAngles().getPitch());
        dashboardTelemetry.addData("roll",imu.getRobotYawPitchRollAngles().getRoll());
        myMotor.setPower(-imu.getRobotYawPitchRollAngles().getYaw());
        if( imu.getRobotYawPitchRollAngles().getYaw()> 0){
            myMotor.setPower(-DRIVE_POWER);
        }else{
            myMotor.setPower(DRIVE_POWER);
        }
        dashboardTelemetry.addData("Drive Power", DRIVE_POWER);
        dashboardTelemetry.update();
        dashboardTelemetry.addData("motor ticks", myMotor.getCurrentPosition());
        dashboardTelemetry.addData("Status", "Run Time: " + runtime.toString());
        myMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myMotor.setTargetPosition(positon);
        dashboardTelemetry.addData("position",positon);
        dashboardTelemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        myMotor.setPower(0);

    }
}