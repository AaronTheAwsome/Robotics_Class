package org.firstinspires.ftc.teamcode.Teleops;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "S")
public class TestOp extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    public static double DRIVE_POWER = 0;
    public static double LAUNCH_POWER = 0;
    public static int position = 10000;
    IMU imu;
    GamepadEx g1;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    DcMotor launchMotor;
    DcMotor myMotor2;
    DcMotor myMotor3;
    DcMotor launchMotor2;
    Servo myServo;
    Servo myServo2;


    @Override
    public void init() {
        g1 = new GamepadEx(gamepad1);
        myServo = hardwareMap.get(Servo.class, "myServo");
        myServo2 = hardwareMap.get(Servo.class, "myServo2");
        imu = hardwareMap.get(IMU.class,"imu");
        launchMotor = hardwareMap.get(DcMotor.class,"launchMotor");
        launchMotor2 = hardwareMap.get(DcMotor.class,"launchMotor2");
        myMotor2 = hardwareMap.get(DcMotor.class,"myMotor2");
        myMotor3 = hardwareMap.get(DcMotor.class,"myMotor3");
        launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor3.setDirection(DcMotor.Direction.REVERSE);
        myMotor2.setDirection(DcMotor.Direction.FORWARD);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        dashboardTelemetry.addData("Status", "Initialized");
        dashboardTelemetry.update();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        runtime.reset();
        imu.resetYaw();
    }

    @Override
    public void loop() {
        g1.readButtons();

        double drive = gamepad1.left_stick_y;
        double turn  = gamepad1.right_stick_x;

        double leftPower  = Range.clip(drive + turn, -1.0, 1.0);
        double rightPower = Range.clip(drive - turn, -1.0, 1.0);
        if (gamepad1.a) {
            myServo.setPosition(0.0);
            myServo2.setPosition(0.0);// Move to position 0 (fully left)
        } else if (gamepad1.b) {
            myServo.setPosition(1.0);
            myServo2.setPosition(1.0);// Move to position 1 (fully right)
        } else if (gamepad1.x) {
            myServo.setPosition(0.5);
            myServo2.setPosition(0.5);// Move to center
        }

        myMotor2.setPower(rightPower);
        myMotor3.setPower(leftPower);
        launchMotor2.setPower(-LAUNCH_POWER);
        // The lanch power for both
        launchMotor.setPower(LAUNCH_POWER);

        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double pitch = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
        double roll = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
        if (Math.abs(turn) < 0.05 ) {
            if (yaw < -2) {
                myMotor2.setPower(-yaw / 10.0);
                myMotor3.setPower(yaw / 10.0);
            } else if (yaw > 2) {
                myMotor2.setPower(-yaw / 10.0);
                myMotor3.setPower(yaw / 10.0);
            }
        }else{
            imu.resetYaw();
        }

        // Telemetry
        dashboardTelemetry.addData("yaw", yaw);
        dashboardTelemetry.addData("pitch", pitch);
        dashboardTelemetry.addData("roll", roll);
        dashboardTelemetry.addData("position", position);
        dashboardTelemetry.addData("Lanch Power", LAUNCH_POWER);
        dashboardTelemetry.addData("Drive Power", DRIVE_POWER);
        dashboardTelemetry.addData("motor ticks", launchMotor.getCurrentPosition());
        dashboardTelemetry.addData("motor 2 ticks", myMotor2.getCurrentPosition());
        dashboardTelemetry.addData("motor 3 ticks", myMotor3.getCurrentPosition());
        dashboardTelemetry.addData("Status", "Run Time: " + runtime.toString());
        dashboardTelemetry.addData("Servo Position", myServo.getPosition());

        dashboardTelemetry.update();
    }

    @Override
    public void stop() {
        launchMotor.setPower(0);
        launchMotor2.setPower(0);
        myMotor2.setPower(0);
        myMotor3.setPower(0);
    }
}

