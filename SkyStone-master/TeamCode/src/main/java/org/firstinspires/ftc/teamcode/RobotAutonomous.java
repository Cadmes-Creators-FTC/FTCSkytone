package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class RobotAutonomous extends LinearOpMode {

    private DcMotor wheelLF;
    private DcMotor wheelRF;
    private DcMotor wheelRB;
    private DcMotor wheelLB;
    private Servo pickupBlockServo;

    private float pickupBlockRotationSpeed = 0.004f;
    private double driveSpeed = 0.8;

    @Override
    public void runOpMode(){

        waitForStart();
        ArmUp();

        while (opModeIsActive()){

        }

    }

    public void DriveForward (double power){

    }

    public void DriveLeft (double power){

    }

    public void TurnLeft (double power){

    }

    public void Armdown (){

    }

    public void ArmUp (){

    }
}
