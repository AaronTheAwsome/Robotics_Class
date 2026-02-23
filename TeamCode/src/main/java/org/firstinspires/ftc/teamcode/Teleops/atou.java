package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

@Autonomous(name = "atuo")
public class atou extends LinearOpMode{
    public void runOpMode(){
        waitForStart();
        int COUNT = 3;
        MecanumDriveTrain myDriveTrain = new MecanumDriveTrain(hardwareMap);
        Shooter SUPA_GUN = new Shooter(hardwareMap);
        myDriveTrain.mecanumDrive(1,0,0);

        //this is the motor and servo code to launch the balls autonumusly
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
