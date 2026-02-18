package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
public class DriveTrain {
    private IMU imu;
    private DcMotorEx myMotor2;
    private DcMotorEx myMotor3;
    private double targetHeading;
    private double currentHeading = 0;

    public static double P_TURN_GAIN = 0.02; // Larger is more responsive, but also less stable.
    public static double P_DRIVE_GAIN = 0.03; // Larger is more responsive, but also less stable.

    public DriveTrain(HardwareMap hardwareMap){

        myMotor2 = hardwareMap.get(DcMotorEx.class,"myMotor2");
        myMotor3 = hardwareMap.get(DcMotorEx.class, "myMotor3");
        myMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        myMotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.resetYaw();
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        dashboardTelemetry.addData("Status", "Initialized");
        dashboardTelemetry.addData("launchSpeed","launchMotor");
        dashboardTelemetry.update();
    }


    public void moveRobot(double drive, double turn) {
        // if turn is zero, hold heading
        if (turn == 0) {
            //is the robot currently moving
            if (drive != 0) {
                turn = getSteeringCorrection(currentHeading, P_DRIVE_GAIN);
            } else {
                turn = getSteeringCorrection(currentHeading, P_TURN_GAIN);
            }
        } else {
            currentHeading = getHeading();
        }

        double leftPower  = Range.clip(drive + turn, -1.0, 1.0);
        double rightPower = Range.clip(drive - turn, -1.0, 1.0);

        // Scale speeds down if either one exceeds +/- 1.0;


        myMotor2.setPower(leftPower);
        myMotor3.setPower(rightPower);

        if (Math.abs(turn) < 0.07 ) {
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
    }

}