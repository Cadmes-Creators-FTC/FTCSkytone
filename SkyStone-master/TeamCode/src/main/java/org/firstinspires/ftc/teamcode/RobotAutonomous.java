package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name="RobotAutonomous", group="MainGroup")
public class RobotAutonomous extends LinearOpMode {

    //variables

    //drive
    private double driveSpeed = 0.8;
    private DcMotor wheelLF;
    private DcMotor wheelRF;
    private DcMotor wheelRB;
    private DcMotor wheelLB;

    //pickup block
    private Servo pickupBlockServo;


    @Override
    public void runOpMode() throws InterruptedException{

        //map wheels
        wheelLF = hardwareMap.get(DcMotor.class, "WheelLF");
        wheelRF = hardwareMap.get(DcMotor.class, "WheelRF");
        wheelRB = hardwareMap.get(DcMotor.class, "WheelRB");
        wheelLB = hardwareMap.get(DcMotor.class, "WheelLB");

        //map servo
        pickupBlockServo = hardwareMap.get(Servo.class, "PickupBlockServo");

        //wait for pressing play
        waitForStart();

        //autonomous sequence
        DriveForwardDistance(1, 10000);
    }



    //driving methods

    //drive forward or backward with distance
    public void DriveForwardDistance (double power, int distance){
        //reset encoders
        wheelLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        wheelLF.setTargetPosition(distance);
        wheelRF.setTargetPosition(distance);
        wheelRB.setTargetPosition(distance);
        wheelLB.setTargetPosition(distance);

        //set to run to position mode
        wheelLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set drive power
        DriveForward(power);

        //wait till distance is reached
        while (wheelLF.isBusy() && wheelRF.isBusy() && wheelRB.isBusy() && wheelLB.isBusy()){}

        //stop driving
        StopDriving();

        //set mode to normal
        wheelLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //drive forward and backward
    public void DriveForward(double power){
        //set power
        power *= driveSpeed;

        //set wheels
        wheelLF.setPower(power);
        wheelRF.setPower(power);
        wheelRB.setPower(power);
        wheelLB.setPower(power);
    }

    //drive left or right with distance
    public void DriveLeftDistance (double power, int distance){
        //reset encoders
        wheelLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        wheelLF.setTargetPosition(-distance);
        wheelRF.setTargetPosition(distance);
        wheelRB.setTargetPosition(-distance);
        wheelLB.setTargetPosition(distance);

        //set to run to position mode
        wheelLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set drive power
        DriveLeft(power);

        //wait till distance is reached
        while (wheelLF.isBusy() && wheelRF.isBusy() && wheelRB.isBusy() && wheelLB.isBusy()){}

        //stop driving
        StopDriving();

        //set mode to normal
        wheelLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //drive left or right
    public void DriveLeft (double power){
        //set power
        power *= driveSpeed;

        //set wheels
        wheelLF.setPower(-power);
        wheelRF.setPower(power);
        wheelRB.setPower(-power);
        wheelLB.setPower(power);
    }

    //turn left or right with distance
    public void TurnLeftDistance (double power, int distance){
        //reset encoders
        wheelLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        wheelLF.setTargetPosition(-distance);
        wheelRF.setTargetPosition(distance);
        wheelRB.setTargetPosition(distance);
        wheelLB.setTargetPosition(-distance);

        //set to run to position mode
        wheelLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set drive power
        TurnLeft(power);

        //wait till distance is reached
        while (wheelLF.isBusy() && wheelRF.isBusy() && wheelRB.isBusy() && wheelLB.isBusy()){}

        //stop driving
        StopDriving();

        //set mode to normal
        wheelLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //turn left or right
    public void TurnLeft (double power){
        //set power
        power *= driveSpeed;

        //set wheels
        wheelLF.setPower(-power);
        wheelRF.setPower(power);
        wheelRB.setPower(power);
        wheelLB.setPower(-power);
    }

    //stop driving
    public void StopDriving(){
        //set wheels
        wheelLF.setPower(0);
        wheelRF.setPower(0);
        wheelRB.setPower(0);
        wheelLB.setPower(0);
    }



    //arm methods

    //set the arm down
    public void Armdown (){
        //turn the servo down
        pickupBlockServo.setPosition(0);

        //wait
        sleep(2000);
    }

    //set the arm up
    public void ArmUp (){
        //turn the servo up
        pickupBlockServo.setPosition(1);

        //wait
        sleep(2000);
    }
}
