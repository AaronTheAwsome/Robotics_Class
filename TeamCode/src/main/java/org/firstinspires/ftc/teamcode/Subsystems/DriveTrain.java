package org.firstinspires.ftc.teamcode.Subsystems;

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

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
public class DriveTrain {
    IMU imu;
    GamepadEx g1;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public static double DRIVE_POWER = 0;
    DcMotor myMotor2;
    DcMotor myMotor3;

    public DriveTrain(HardwareMap hardwareMap){
        myMotor2 = hardwareMap.get(DcMotorEx.class,"left_drive");
        myMotor3 = hardwareMap.get(DcMotorEx.class, "right_drive");

        myMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        myMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        myMotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        myMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        myMotor3.setDirection(DcMotor.Direction.REVERSE);
        myMotor2.setDirection(DcMotor.Direction.FORWARD);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        dashboardTelemetry.addData("Status", "Initialized");
        dashboardTelemetry.update();
    }


    public void moveRobot(double drive, double turn) {
        g1.readButtons();

        double leftPower  = Range.clip(drive + turn, -1.0, 1.0);
        double rightPower = Range.clip(drive - turn, -1.0, 1.0);

        myMotor2.setPower(rightPower);
        myMotor3.setPower(leftPower);
        // The launch power for both


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
        dashboardTelemetry.addData("Drive Power", DRIVE_POWER);
        dashboardTelemetry.addData("motor 2 ticks", myMotor2.getCurrentPosition());
        dashboardTelemetry.addData("motor 3 ticks", myMotor3.getCurrentPosition());

        dashboardTelemetry.update();
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
