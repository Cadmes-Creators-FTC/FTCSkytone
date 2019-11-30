package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



@TeleOp(name = "Main_Robot", group = "MainGroup")
public class Main_Robot extends LinearOpMode {

    //drive
    private DcMotor wheelLF;
    private DcMotor wheelRF;
    private DcMotor wheelRB;
    private DcMotor wheelLB;

    @Override
    public void runOpMode (){

        telemetry.addData("running?", "true");
        telemetry.update();

        MapHardware();

        waitForStart();

        while (opModeIsActive()){

            //drive and turn
            DriveWithController();
        }

    }



    //map hardware
    private void MapHardware(){
        //drive
        wheelLF = hardwareMap.get(DcMotor.class, "WheelLF");
        wheelRF = hardwareMap.get(DcMotor.class, "WheelRF");
        wheelRB = hardwareMap.get(DcMotor.class, "WheelRB");
        wheelLB = hardwareMap.get(DcMotor.class, "WheelLB");

        //set wheel encoder mode
        wheelLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //reverse wheels
        wheelRF.setDirection(DcMotor.Direction.REVERSE);
        wheelRB.setDirection(DcMotor.Direction.REVERSE);
    }

    private void DriveWithController(){
        double joyX = gamepad1.left_stick_x;
        double joyY = gamepad1.left_stick_y;
        double joyR = gamepad1.right_stick_x;

        double inputLF = 0;
        double inputRF = 0;
        double inputLB = 0;
        double inputRB = 0;

        inputLF -= joyX;
        inputRF += joyX;
        inputLB += joyX;
        inputRB -= joyX;

        inputLF += joyY;
        inputRF += joyY;
        inputLB += joyY;
        inputRB += joyY;

        inputLF -= joyR;
        inputRF += joyR;
        inputLB -= joyR;
        inputRB += joyR;

        telemetry.addData("LF", inputLF);
        telemetry.addData("RF", inputRF);
        telemetry.addData("RB", inputRB);
        telemetry.addData("LB", inputLB);
        telemetry.update();

        wheelLF.setPower(inputLF);
        wheelRF.setPower(inputRF);
        wheelLB.setPower(inputLB);
        wheelRB.setPower(inputRB);
    }
}
