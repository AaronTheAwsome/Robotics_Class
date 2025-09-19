package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp (name = "Trial 40")
//@Autonomous
public class TestOp extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    public static double DRIVE_POWER = 0;
    public static double ARM_POWER = 0;
    public static int positon = 10000;
    IMU imu;
    GamepadEx g1;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    DcMotor armMotor;
    DcMotor myMotor2;
    DcMotor myMotor3;

    @Override
    public void init() {
        g1 = new GamepadEx(gamepad1);
        armMotor = hardwareMap.get(DcMotor.class,"armMotor");
        myMotor2 = hardwareMap.get(DcMotor.class,"myMotor2");
        myMotor3 = hardwareMap.get(DcMotor.class,"myMotor3");
        imu = hardwareMap.get(IMU.class,"imu");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor3.setDirection(DcMotor.Direction.REVERSE);
        myMotor2.setDirection(DcMotor.Direction.FORWARD);
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
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


        double drive = -gamepad1.left_stick_y;
        double turn  = gamepad1.right_stick_x;
        double leftPower  = Range.clip(drive + turn, -1.0, 1.0);
        double rightPower = Range.clip(drive - turn, -1.0, 1.0);
        armMotor.setPower(ARM_POWER);
        myMotor3.setPower(leftPower);
        myMotor2.setPower(rightPower);
        if (Math.abs(turn)>0) {
            imu.resetYaw();
        }


        dashboardTelemetry.addData("yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        dashboardTelemetry.addData("pitch", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
        dashboardTelemetry.addData("roll", imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));

        if( imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)< -2){
            myMotor2.setPower(-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)/10);
            myMotor3.setPower(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)/10);
            if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < -5){
                stop();
                myMotor2.setPower(0);
                myMotor3.setPower(0);
            }
        } else if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > 2) {
            myMotor2.setPower(-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)/10);
            myMotor3.setPower(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)/10);
            if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > 5){
                stop();
                myMotor2.setPower(0);
                myMotor3.setPower(0);
            }
        } else {
            myMotor2.setPower(DRIVE_POWER);
            myMotor3.setPower(DRIVE_POWER);
        }

        dashboardTelemetry.addData("Drive Power", DRIVE_POWER);
        dashboardTelemetry.addData("Arm Power",ARM_POWER);
        dashboardTelemetry.addData("motor ticks", armMotor.getCurrentPosition());
        dashboardTelemetry.addData("motor 2 ticks",myMotor2.getCurrentPosition());
        dashboardTelemetry.addData("motor 3 ticks",myMotor3.getCurrentPosition());
        dashboardTelemetry.addData("Status", "Run Time: " + runtime.toString());
        dashboardTelemetry.addData("position",positon);
        dashboardTelemetry.update();
    }

    @Override
    public void stop() {
        armMotor.setPower(0);
        myMotor2.setPower(0);
        myMotor3.setPower(0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
