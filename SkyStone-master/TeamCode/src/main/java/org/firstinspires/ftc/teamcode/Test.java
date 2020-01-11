package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Test", group = "Test")
public class Test extends LinearOpMode {


    @Override
    public void runOpMode () {


        MapHardware();

        waitForStart();

        while (opModeIsActive()) {

        }
    }


    //map hardware
    private void MapHardware(){

    }
}
