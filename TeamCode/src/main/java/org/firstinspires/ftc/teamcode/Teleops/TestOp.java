package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "BattleDriod Manual Control")
public class TestOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    public static double DRIVE_POWER = 0;
    public static double LAUNCH_POWER = 0;
    public static int position = 10000;

    IMU imu;
    GamepadEx g1;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    DcMotor liftMotor;
    DcMotor myMotor2;
    DcMotor myMotor3;
    DcMotor pickUp;
    DcMotor myMotorE;
    DcMotor myMotorE2;
    Servo liftServo;
    Servo liftServo2;

    @Override
    public void init() {

        g1 = new GamepadEx(gamepad1);

        imu = hardwareMap.get(IMU.class,"imu");
        liftMotor = hardwareMap.get(DcMotor.class,"liftMotor");
        pickUp = hardwareMap.get(DcMotor.class,"pickUp");
        myMotor2 = hardwareMap.get(DcMotor.class,"myMotor2");
        myMotor3 = hardwareMap.get(DcMotor.class,"myMotor3");
        myMotorE= hardwareMap.get(DcMotor.class,"myMotorE");
        myMotorE2= hardwareMap.get(DcMotor.class,"myMotorE2");
        liftServo = hardwareMap.get(Servo.class,"liftServo");
        liftServo2= hardwareMap.get(Servo.class,"liftServo2");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pickUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotorE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotorE2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pickUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotorE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotorE2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        myMotor3.setDirection(DcMotor.Direction.REVERSE);
        myMotor2.setDirection(DcMotor.Direction.FORWARD);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        runtime.reset();

        dashboardTelemetry.addData("Status", "Initialized");
        dashboardTelemetry.addData("launchSpeed","launchMotor");
        dashboardTelemetry.update();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        liftServo.setPosition(0.5);
        liftServo2.setPosition(0.5);
    }

    @Override
    public void loop() {
        g1.readButtons();

        double drive = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double leftPower = Range.clip(drive + turn, -1.0, 1.0);
        double rightPower = Range.clip(drive - turn, -1.0, 1.0);

        if (gamepad1.x) {
            pickUp.setPower(-1);
            liftMotor.setPower(0.2);
        } else if (gamepad1.circle){
            pickUp.setPower(0);
            liftMotor.setPower(0);
        }

        if (gamepad1.triangle){
            liftServo.setPosition(0);
            liftServo2.setPosition(1);
        } else {
            liftServo.setPosition(0.5);
            liftServo2.setPosition(0.5);
        }

        // note: using GamepadEx for edge-detect
        if (g1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
            liftMotor.setPower(1);
        }
        if (g1.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)) {
            liftMotor.setPower(0);
        }

        if (g1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
            myMotorE.setPower(1);
            myMotorE2.setPower(-1);
        }
        if (g1.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
            myMotorE.setPower(0);
            myMotorE2.setPower(0);
        }

        myMotor2.setPower(rightPower);
        myMotor3.setPower(leftPower);

        double yaw   = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double pitch = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
        double roll  = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);

        if (Math.abs(turn) < 0.07 ) {
            if (yaw < -2 || yaw > 2) {
                myMotor2.setPower(-yaw / 50.0);
                myMotor3.setPower(yaw / 50.0);
            }
        } else {
            imu.resetYaw();
        }

        dashboardTelemetry.addData("yaw", yaw);
        dashboardTelemetry.addData("pitch", pitch);
        dashboardTelemetry.addData("roll", roll);
        dashboardTelemetry.addData("position", position);
        dashboardTelemetry.addData("Launch Power", LAUNCH_POWER);
        dashboardTelemetry.addData("Drive Power", DRIVE_POWER);
        dashboardTelemetry.addData("motor ticks", liftMotor.getCurrentPosition());
        dashboardTelemetry.addData("motor 2 ticks", myMotor2.getCurrentPosition());
        dashboardTelemetry.addData("motor 3 ticks", myMotor3.getCurrentPosition());
        dashboardTelemetry.addData("Status", "Run Time: " + runtime.toString());
        dashboardTelemetry.update();
    }

    @Override
    public void stop() {
        liftMotor.setPower(0);
        pickUp.setPower(0);
        myMotor2.setPower(0);
        myMotor3.setPower(0);
        myMotorE.setPower(0);
        myMotorE2.setPower(0);
    }
}
