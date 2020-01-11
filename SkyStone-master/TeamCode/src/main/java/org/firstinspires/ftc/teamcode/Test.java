package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "Test", group = "MainGroup")
public class Test extends LinearOpMode {

    //drive
    private DcMotor wheelLF;
    private DcMotor wheelRF;
    private DcMotor wheelRB;
    private DcMotor wheelLB;

    @Override
    public void runOpMode () {


        MapHardware();

        waitForStart();

        while (opModeIsActive()) {

            //update telemetry
            telemetry.update();
        }

        // turn the motors off.
        wheelLF.setPower(0);
        wheelRF.setPower(0);
        wheelRB.setPower(0);
        wheelLB.setPower(0);
    }



    //map hardware
    private void MapHardware(){
    }
}
