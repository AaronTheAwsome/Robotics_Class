package org.firstinspires.ftc.teamcode.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Crappy Stuff")
//@Disabled
public class TeleOp extends OpMode {
    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;

    // Declare OpMode members.
    MecanumDriveTrain myDriveTrain;
    private Shooter myShooter;
    GamepadEx g1;





    ElapsedTime feederTimer = new ElapsedTime();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    @Override
    public void init() {


        this.myDriveTrain = new MecanumDriveTrain(hardwareMap);
        this.myShooter = new Shooter(hardwareMap);
        g1 = new GamepadEx(gamepad1);

        telemetry.addData("Status", "Initialized");
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

    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        g1.readButtons();
        if (g1.wasJustPressed(GamepadKeys.Button.B)){
            myShooter.toggleMotor();

        }else{

        }


        if (g1.wasJustPressed(GamepadKeys.Button.X)){
            myShooter.servopos2();
        }
        else if (g1.wasJustReleased(GamepadKeys.Button.X)) {
            myShooter.servopos1();
        }

        myDriveTrain.mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);


    }


    @Override
    public void stop() {
    }



}
