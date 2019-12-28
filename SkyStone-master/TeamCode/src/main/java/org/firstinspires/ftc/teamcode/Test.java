package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Test", group = "MainGroup")
public class Test extends LinearOpMode {


    //IMU
    private BNO055IMU IMU;

    @Override
    public void runOpMode (){

        telemetry.addData("running?", "true");
        telemetry.update();

        MapHardware();

        waitForStart();

        while (opModeIsActive()){

            telemetry.addData("position", IMU.getPosition());

            //update telemetry
            telemetry.update();
        }

    }



    //map hardware
    private void MapHardware(){

        //imu
        IMU = hardwareMap.get(BNO055IMU.class, "imu");
    }
}
