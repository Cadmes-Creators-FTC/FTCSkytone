package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Test", group = "Test")
public class Test extends LinearOpMode {

    private DcMotor RightForward;
    private DcMotor LeftForward;
    private DcMotor RightBack;
    private DcMotor LeftBack;




    @Override
    public void runOpMode () {

        telemetry.addData("State", "initialized");
        telemetry.update();

        MapHardware();

        waitForStart();

        while (opModeIsActive()){

            telemetry.addData("State", "Running");
            telemetry.update();

            float JoyStickY = gamepad1.left_stick_y;
            LeftForward.setPower(JoyStickY);
            LeftBack.setPower(JoyStickY);
            RightForward.setPower(JoyStickY);
            RightBack.setPower(JoyStickY);


        }

        telemetry.addData("State", "Disabled");
        telemetry.update();

    }


    //map hardware
    private void MapHardware(){
    RightForward = hardwareMap.get(DcMotor.class,  "RightForward");
    LeftForward = hardwareMap.get(DcMotor.class,  "LeftForward");
    RightBack = hardwareMap.get(DcMotor.class,  "RightBack");
    LeftBack = hardwareMap.get(DcMotor.class,  "LeftBack");

    RightBack.setDirection(DcMotor.Direction.REVERSE);
    RightForward.setDirection(DcMotor.Direction.REVERSE);


    }
}
