package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class Main_Robot extends LinearOpMode {

    //drive
    private double driveSpeed = 0.8;
    private DcMotor wheelLF;
    private DcMotor wheelRF;
    private DcMotor wheelRB;
    private DcMotor wheelLB;



    //pickupblock
    private Servo pickupBlockServo;
    private float pickupBlockRotation = 0;
    private float pickupBlockRotationSpeed = 0.004f;

    //pickupplateservo
    private Servo pickupPlateServo;
    private float pickupPlateRotation = 0;
    private float pickupPlateRotationSpeed = 0.004f;


    @Override
    public void runOpMode (){

        telemetry.addData("running?", "true");
        telemetry.update();

        MapHardware();

        waitForStart();

        while (opModeIsActive()){
            //change speed
            ChangeSpeed();

            //drive and turn
            DriveWithController();

            //turn the pickupblock servo
            PickupBlock();

            //turn the pickupplate servo
            PickupPlate();

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

        //pickup plate
        pickupPlateServo = hardwareMap.get(Servo.class, "PickupPlateServo");

        //reverse wheels
        wheelRF.setDirection(DcMotor.Direction.REVERSE);
        wheelRB.setDirection(DcMotor.Direction.REVERSE);
    }



    private void ChangeSpeed(){

        //change the drive speed
        if(gamepad1.y){
            driveSpeed = 0.3;
        }else if (gamepad1.b){
            driveSpeed = 0.5;
        }else if (gamepad1.a){
            driveSpeed = 0.7;
        }else if (gamepad1.x){
            driveSpeed = 0.9;
        }
    }


    private void DriveWithController(){
        double joyX = gamepad1.left_stick_x * driveSpeed;
        double joyY = gamepad1.left_stick_y * driveSpeed;
        double joyR = gamepad1.right_stick_x * driveSpeed;

        wheelLF.setPower(-joyX + joyY - joyR);
        wheelRF.setPower(joyX + joyY + joyR);
        wheelLB.setPower(joyX + joyY - joyR);
        wheelRB.setPower(-joyX + joyY + joyR);
    }



    //pickup block
    private void PickupBlock(){

        if(gamepad2.left_stick_y > 0.1){
            //turn the servo down
            pickupBlockRotation += pickupBlockRotationSpeed;

        }else if(gamepad2.left_stick_y < -0.1){
            //turn the servo up
            pickupBlockRotation -= pickupBlockRotationSpeed;

        }

        pickupBlockServo.setPosition(pickupBlockRotation);
    }



    //pickup block
    private void PickupPlate(){

        if(gamepad2.right_stick_y > 0.1){
            //turn the servo down
            pickupPlateRotation += pickupPlateRotationSpeed;

        }else if(gamepad2.right_stick_y < -0.1){
            //turn the servo up
            pickupPlateRotation -= pickupPlateRotationSpeed;

        }

        pickupPlateServo.setPosition(pickupPlateRotation);
    }

}
