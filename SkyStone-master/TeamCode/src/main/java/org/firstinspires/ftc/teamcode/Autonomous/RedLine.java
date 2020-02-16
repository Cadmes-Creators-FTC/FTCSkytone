package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotsConfigs.Robot;


@SuppressWarnings({"RedundantThrows", "SameParameterValue", "unused", "FieldCanBeLocal"})
@Autonomous (name="RedLine", group="Autonomous")
public class RedLine extends LinearOpMode {

    private Robot robot;


    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData("State", "initializing");
        telemetry.update();

        //initialize robot hardware
        robot = new Robot(hardwareMap, telemetry);

        telemetry.addData("State", "initialized");
        telemetry.addData("imu calibration state: ", "calibrating");
        telemetry.update();

        //wait for imu to calibrate
        robot.WaitForGyroCalibration();

        telemetry.addData("State", "initialized");
        telemetry.addData("imu calibration state: ", "calibrated");
        telemetry.update();

        //wait for start button to be pressed
        waitForStart();

        //start autonomous
        AutonomousSequence();

        telemetry.addData("State", "Done");
        telemetry.update();
    }

    //autonomous sequence
    private void AutonomousSequence(){
        robot.DriveForward(10, 0.5);
        robot.DriveLeft(90,0.3);
        robot.DriveBackward(20, 0.2);
    }
}