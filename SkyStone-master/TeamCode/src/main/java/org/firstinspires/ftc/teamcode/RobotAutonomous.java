package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@SuppressWarnings({"RedundantThrows", "SameParameterValue", "unused"})
@Autonomous (name="RobotAutonomous", group="MainGroup")
public class RobotAutonomous extends LinearOpMode {

    //variables

    //drive
    private DcMotor wheelLF;
    private DcMotor wheelRF;
    private DcMotor wheelRB;
    private DcMotor wheelLB;


    @Override
    public void runOpMode() throws InterruptedException{

        //map wheels
        wheelLF = hardwareMap.get(DcMotor.class, "WheelLF");
        wheelRF = hardwareMap.get(DcMotor.class, "WheelRF");
        wheelRB = hardwareMap.get(DcMotor.class, "WheelRB");
        wheelLB = hardwareMap.get(DcMotor.class, "WheelLB");

        //set wheels to brake
        wheelLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reverse wheels
        wheelLF.setDirection(DcMotor.Direction.REVERSE);
        wheelLB.setDirection(DcMotor.Direction.REVERSE);

        //wait for pressing play
        waitForStart();

        //if on start autonomous
        if(opModeIsActive())
            AutonomousSequence();
    }

    private void AutonomousSequence(){
        //autonomous sequence
        DriveForwardTime(1, 300);
        DriveBackwardTime(1, 2000);
        StopDriving();
    }

    //driving methods

    //drive forward with time
    private void DriveForwardTime (double power, int time){
        //stop if disabled
        if(!opModeIsActive())
            return;

        //set wheels
        wheelLF.setPower(power);
        wheelRF.setPower(power);
        wheelRB.setPower(power);
        wheelLB.setPower(power);

        //wait
        sleep(time);
    }

    //drive backward with time
    private void DriveBackwardTime(double power, int time){
        //stop if disabled
        if(!opModeIsActive())
            return;

        //set wheels
        wheelLF.setPower(-power);
        wheelRF.setPower(-power);
        wheelRB.setPower(-power);
        wheelLB.setPower(-power);

        //wait
        sleep(time);
    }


    //drive left with time
    private void DriveLeftTime (double power, int time){
        //stop if disabled
        if(!opModeIsActive())
            return;

        //set wheels
        wheelLF.setPower(power);
        wheelRF.setPower(-power);
        wheelRB.setPower(power);
        wheelLB.setPower(-power);

        //wait
        sleep(time);
    }

    //drive right with time
    private void DriveRightTime (double power, int time){
        //stop if disabled
        if(!opModeIsActive())
            return;

        //set wheels
        wheelLF.setPower(-power);
        wheelRF.setPower(power);
        wheelRB.setPower(-power);
        wheelLB.setPower(power);

        //wait
        sleep(time);
    }


    //turn left with time
    private void TurnLeftTime (double power, int time){
        //stop if disabled
        if(!opModeIsActive())
            return;

        //set wheels
        wheelLF.setPower(-power);
        wheelRF.setPower(power);
        wheelRB.setPower(power);
        wheelLB.setPower(-power);

        //wait
        sleep(time);
    }

    //turn right with time
    private void TurnRightTime (double power, int time){
        //stop if disabled
        if(!opModeIsActive())
            return;

        //set wheels
        wheelLF.setPower(power);
        wheelRF.setPower(-power);
        wheelRB.setPower(-power);
        wheelLB.setPower(power);

        //wait
        sleep(time);
    }


    //stop driving
    private void StopDriving(){
        //set wheels
        wheelLF.setPower(0);
        wheelRF.setPower(0);
        wheelRB.setPower(0);
        wheelLB.setPower(0);
    }
}
