package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class Main_Robot extends LinearOpMode {

    //drive
    private double speed = 0.8;
    private DcMotor wheelLF;
    private DcMotor wheelRF;
    private DcMotor wheelRB;
    private DcMotor wheelLB;



    //pickup block
    private Servo pickupBlockServo;
    private float pickupRotation = 0;
    private float pickupRotationSpeed = 0.004f;


    @Override
    public void runOpMode (){

        telemetry.addData("running?", "true");
        telemetry.update();

        MapHardware();

        waitForStart();

        while (opModeIsActive()){
            //drive and turn
            driveWithController();

            //turn the pickup servo
            PickupBlock();

            telemetry.update();
        }

    }



    //maphardware
    private void MapHardware(){
        //drive
        wheelLF = hardwareMap.get(DcMotor.class, "WheelLF");
        wheelRF = hardwareMap.get(DcMotor.class, "WheelRF");
        wheelRB = hardwareMap.get(DcMotor.class, "WheelRB");
        wheelLB = hardwareMap.get(DcMotor.class, "WheelLB");

        //pickup block
        pickupBlockServo = hardwareMap.get(Servo.class, "PickupBlockServo");

        //reverse wheels
        wheelRF.setDirection(DcMotor.Direction.REVERSE);
        wheelRB.setDirection(DcMotor.Direction.REVERSE);
    }



    //Nigel Was Here [10918]
    private void driveWithController(){
        double joyX = gamepad1.left_stick_x * speed;
        double joyY = gamepad1.left_stick_y * speed;
        double joyR = gamepad1.right_stick_x * speed;

        wheelLF.setPower(joyX + joyY + joyR);
        wheelRF.setPower(-joyX + joyY - joyR);
        wheelLB.setPower(-joyX + joyY + joyR);
        wheelRB.setPower(joyX + joyY - joyR);
    }



    //pickup block
    private void PickupBlock(){

        if(gamepad1.right_trigger > 0.5f || gamepad2.right_trigger > 0.5f){
            //turn the servo down
            pickupRotation += pickupRotationSpeed;

        }else if(gamepad1.right_bumper || gamepad2.right_bumper){
            //turn the servo up
            pickupRotation -= pickupRotationSpeed;

        }

        pickupBlockServo.setPosition(pickupRotation);
    }

}
