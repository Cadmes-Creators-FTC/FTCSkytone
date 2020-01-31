package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Test", group = "Test")
public class Test extends LinearOpMode {


    @Override
    public void runOpMode () {

        telemetry.addData("State", "initialized");
        telemetry.update();

        MapHardware();

        waitForStart();

        while (opModeIsActive()){

            telemetry.addData("State", "Running");
            telemetry.update();

        }

        telemetry.addData("State", "Disabled");
        telemetry.update();

    }


    //map hardware
    private void MapHardware(){

    }
}
