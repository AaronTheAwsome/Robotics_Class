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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Config
@TeleOp (name = "Trial 30")
//@Autonomous
public class TestOp extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    public static double DRIVE_POWER = 0;
    public static int positon = 10000;
    IMU imu;
    GamepadEx g1;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    DcMotor myMotor;
    DcMotor myMotor2;
    DcMotor myMotor3;

    @Override
    public void init() {
        g1 = new GamepadEx(gamepad1);

        myMotor = hardwareMap.get(DcMotor.class,"myMotor");
        myMotor2 = hardwareMap.get(DcMotor.class,"myMotor2");
        myMotor3 = hardwareMap.get(DcMotor.class,"myMotor3");
        imu = hardwareMap.get(IMU.class,"imu");
        myMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        myMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        myMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);// Turn the motor back on when we are done
        myMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        myMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor3.setDirection(DcMotor.Direction.REVERSE);
        myMotor2.setDirection(DcMotor.Direction.FORWARD);// Turn the motor back on when we are done
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        double drive = -gamepad1.left_stick_y;
        double turn  =  -gamepad1.right_stick_x;
        double leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        double rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
        myMotor3.setPower(leftPower);
        myMotor2.setPower(rightPower);
        // Tell the driver that initialization is complete.
        dashboardTelemetry.addData("Status", "Initialized");
        dashboardTelemetry.update();

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
        imu.resetYaw();
    }

    @Override
    public void loop() {
        g1.readButtons();
        dashboardTelemetry.addData("yaw",imu.getRobotYawPitchRollAngles().getYaw());
        dashboardTelemetry.addData("pitch",imu.getRobotYawPitchRollAngles().getPitch());
        dashboardTelemetry.addData("roll",imu.getRobotYawPitchRollAngles().getRoll());
        if( imu.getRobotYawPitchRollAngles().getYaw()< -2){
            myMotor.setPower(-imu.getRobotYawPitchRollAngles().getYaw());
            myMotor2.setPower(-imu.getRobotYawPitchRollAngles().getYaw()/10);
            myMotor3.setPower(imu.getRobotYawPitchRollAngles().getYaw()/10);
            if (imu.getRobotYawPitchRollAngles().getYaw() < -5){
                stop();
                myMotor.setPower(0);
                myMotor2.setPower(0);
                myMotor3.setPower(0);
            }
        } else if (imu.getRobotYawPitchRollAngles().getYaw() > 2) {
            myMotor.setPower(imu.getRobotYawPitchRollAngles().getYaw());
            myMotor2.setPower(-imu.getRobotYawPitchRollAngles().getYaw()/10);
            myMotor3.setPower(imu.getRobotYawPitchRollAngles().getYaw()/10);
            if (imu.getRobotYawPitchRollAngles().getYaw() > 5){
                stop();
                myMotor.setPower(0);
                myMotor2.setPower(0);
                myMotor3.setPower(0);
            }
        }else{
            myMotor.setPower(0);
            myMotor2.setPower(DRIVE_POWER);
            myMotor3.setPower(DRIVE_POWER);
        }
        dashboardTelemetry.addData("Drive Power", DRIVE_POWER);
        dashboardTelemetry.addData("motor ticks", myMotor.getCurrentPosition());
        dashboardTelemetry.addData("motor 2 ticks",myMotor2.getCurrentPosition());
        dashboardTelemetry.addData("motor 3 ticks",myMotor3.getCurrentPosition());
        dashboardTelemetry.addData("Status", "Run Time: " + runtime.toString());
        myMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myMotor.setTargetPosition(positon);
        //myMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //myMotor2.setTargetPosition(positon);
        //myMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //myMotor3.setTargetPosition(positon);
        dashboardTelemetry.addData("position",positon);
        dashboardTelemetry.update();
    }

    @Override
    public void stop() {
        myMotor.setPower(0);
        myMotor2.setPower(0);
        myMotor3.setPower(0);
        myMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        myMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        myMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);// Turn the motor back on when we are done
        myMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        myMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);// Turn the motor back on when we are done

    }
}