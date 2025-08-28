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
    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private double headingError = 0;
    static final double INCHES_PER_COUNT = 0;
    private double targetHeading;
    private double currentHeading = 0;

    DcMotor myMotor;

    // @Config variables
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    public static double P_TURN_GAIN = 0.02; // Larger is more responsive, but also less stable.
    public static double P_DRIVE_GAIN = 0.03; // Larger is more responsive, but also less stable.

    public DriveTrain(HardwareMap hardwareMap){
        // TODO
        // Make sure you have the motors configured correctly in robot config
        leftMotor = hardwareMap.get(DcMotorEx.class,"left_drive");
        rightMotor = hardwareMap.get(DcMotorEx.class, "right_drive");
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // TODO
        // use the following lines of code to reverse the direction of your motors if needed
        // when given a positive input the robot should move forward
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        // rightMotor.setDirection(DcMotor.Direction.REVERSE);
        myMotor.setPower(0.5);
        // TODO - EDIT these two lines to match YOUR mounting configuration.
        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed
         * logo pointing UP and the USB port pointing FORWARD
         * https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html#configure-imu
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // TODO
        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        // https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html#configure-imu
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
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

        double leftSpeed  = drive - turn;
        double rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        leftMotor.setPower(leftSpeed);
        rightMotor.setPower(rightSpeed);
    }

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
