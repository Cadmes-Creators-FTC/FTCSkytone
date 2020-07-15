package org.firstinspires.ftc.teamcode.Testing;


import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotsConfigs.Robot;

import java.util.Timer;


@Autonomous (name="SlipTest1", group="Autonomous")
public class SlipTest1 extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException{
        //initialize robot hardware
        robot = new Robot(hardwareMap, telemetry);

        //wait for start button to be pressed
        waitForStart();

        new Thread(new Runnable() {
            @Override
            public void run() {
                ShowTicks();
            }
        }).start();

        //start autonomous
        AutonomousSequence();

    }

    //autonomous sequence
    private void AutonomousSequence() throws InterruptedException {
        robot.DriveRight(100, 1);
        Thread.sleep(3000);
        robot.DriveLeft(100, 1);
    }

    private void ShowTicks(){
        telemetry.addData("ticks motorlf", robot.wheelLF.getCurrentPosition());
        telemetry.addData("ticks motorrf", robot.wheelRF.getCurrentPosition());
        telemetry.addData("ticks motorrb", robot.wheelRB.getCurrentPosition());
        telemetry.addData("ticks motorlb", robot.wheelLB.getCurrentPosition());
        telemetry.update();
    }

}