package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
@Autonomous(name = "atuo")
public class atou extends LinearOpMode{
    public void runOpMode(){
        waitForStart();
        int COUNT = 3;
        DriveSubsystem myDriveTrain = new DriveSubsystem(hardwareMap);
        Shooter SUPA_GUN = new Shooter(hardwareMap);
        myDriveTrain.drive2(1,0,0);
        sleep(1000);
        SUPA_GUN.toggleMotor();
        //this is the motor and servo code to launch the balls autonomously
        SUPA_GUN.toggleMotor();
        for (int i = 0; i < COUNT; i++) {
            sleep(1000);
            SUPA_GUN.servopos2();
            sleep(500);
            SUPA_GUN.servopos1();
        }
        SUPA_GUN.toggleMotor();

        sleep(1000);
    }
}
