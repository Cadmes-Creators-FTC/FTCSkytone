package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Main_Robot", group = "MainGroup")
public class Main_Robot extends LinearOpMode {

    //drive
    private double driveSpeed = 0.8;
    private DcMotor wheelLF;
    private DcMotor wheelRF;
    private DcMotor wheelRB;
    private DcMotor wheelLB;



    //pickupblock
    private Servo pickupBlockServo;
    private double pickupBlockServoPosition;
    private DcMotor CraneMotor;


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

            //Move the arm
            Arm();

            //update telemetry
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

        //set encoder mode
        wheelLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //pickup block
        pickupBlockServo = hardwareMap.get(Servo.class, "PickupBlockServo");
        CraneMotor = hardwareMap.get(DcMotor.class, "PickupCrane");

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

    private void Arm(){
        //get the input
        double inputLeftStick = gamepad2.left_stick_y;
        double inputRightStick = gamepad2.right_stick_y;

        double servoRotationSpeed = 0.004f;

        //get servo position
        if(inputLeftStick > 0.1)
            pickupBlockServoPosition += servoRotationSpeed;
        else if(inputLeftStick < -0.1)
            pickupBlockServoPosition -= servoRotationSpeed;

        //set servo position
        pickupBlockServo.setPosition(pickupBlockServoPosition);

        //rotate the motor
        CraneMotor.setPower(inputRightStick * 0.15);
    }
}
