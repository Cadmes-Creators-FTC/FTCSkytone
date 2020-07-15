package org.firstinspires.ftc.teamcode.Testing;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotsConfigs.Robot;


@Autonomous (name="SlipTest3", group="Autonomous")
public class SlipTest3 extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException{
        //initialize robot hardware
        robot = new Robot(hardwareMap, telemetry);

        //wait for start button to be pressed
        waitForStart();

        //start autonomous
        AutonomousSequence();
    }

    //autonomous sequence
    private void AutonomousSequence(){
        robot.DriveForward(100, 1);
    }
}