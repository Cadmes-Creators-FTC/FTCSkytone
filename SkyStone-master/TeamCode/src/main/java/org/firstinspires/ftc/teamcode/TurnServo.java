package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TurnServo extends LinearOpMode {
    String currentstate = "";

    float minrot = 0;
    float maxrot = 1;
    float middlerot = 0.5f;

    private Servo servo_1;

    @Override
    public void runOpMode (){

        servo_1 = hardwareMap.get(Servo.class, "testservo");

        while (opModeIsActive()){

            if(gamepad1.dpad_left || gamepad2.dpad_left){
                //turn the servo to the min rotation
                servo_1.setPosition(minrot);
                currentstate = "minimal rotation";

            }else if(gamepad1.dpad_up || gamepad2.dpad_up){
                //turn the servo to the middle
                servo_1.setPosition(middlerot);
                currentstate = "middle rotation";

            }else if(gamepad1.dpad_right || gamepad2.dpad_right) {
                //turn the servo to the max rotation
                servo_1.setPosition(maxrot);
                currentstate = "maximal rotation";
            }

            telemetry.addData("State", currentstate);
            telemetry.addData("Rotation", servo_1.getPosition());
        }

    }

}
