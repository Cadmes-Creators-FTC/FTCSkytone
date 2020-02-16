package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotsConfigs.Robot;


@SuppressWarnings({"RedundantThrows", "SameParameterValue", "unused", "FieldCanBeLocal"})
@Autonomous (name="BlueBuildPlateLine", group="Autonomous")
public class BlueBuildPlateLine extends LinearOpMode {

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
        robot.DriveForward(40, 0.4);
        robot.DriveLeft(40, 0.4);
        robot.DriveForward(20, 0.4);
        robot.DriveForward(40, 0.2);
        robot.BuildPlateHooksDown();
        robot.DriveBackward(90, 0.4);
        robot.Turn(-90, 0.5);
        robot.BuildPlateHooksUp();
        robot.DriveBackward(40, 0.3);
        robot.Turn(90, 0.5);
        robot.DriveRight(50, 0.3);
        robot.DriveBackward(20, 0.3);
    }
}