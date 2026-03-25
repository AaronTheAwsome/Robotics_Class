package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
@Autonomous(name = "auto")
public class atou extends LinearOpMode{


    public void runOpMode(){
        waitForStart();
        int COUNT = 4;
        DriveSubsystem myDriveTrain = new DriveSubsystem(hardwareMap);
        myDriveTrain.setHeadingToMaintain(myDriveTrain.getCurrentHeadingDeg());
        sleep(1000);
        Shooter SUPA_GUN = new Shooter(hardwareMap);
        myDriveTrain.slowDown(1);
        sleep(100);
        myDriveTrain.slowDown(0.9);
        sleep(100);
        myDriveTrain.slowDown(0.8);
        sleep(100);
        myDriveTrain.slowDown(0.7);
        sleep(100);
        myDriveTrain.slowDown(0.6);
        sleep(100);
        myDriveTrain.slowDown(0.5);
        sleep(100);
        myDriveTrain.slowDown(0.4);
        sleep(100);
        myDriveTrain.slowDown(0.3);
        sleep(100);
        myDriveTrain.slowDown(0.2);
        sleep(100);
        myDriveTrain.slowDown(0.1);
        sleep(100);
        myDriveTrain.stopMotors();
        sleep (1000);
        /*for (int i = 1; i < 10; i++){
            SUPA_GUN.toggleMotor();
        }
        for (int i = 0; i < COUNT; i++) {
            sleep(1000);
            SUPA_GUN.servopos2();
            sleep(500);
            SUPA_GUN.servopos1();
        }
        SUPA_GUN.toggleMotor();*/
        sleep(1000);
    }
}