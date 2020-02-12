package org.firstinspires.ftc.teamcode;

public class MathFunctions {

    //convert cm to encoder ticks
    public static int CMToTicks(double CM, boolean side){
        if(side) {
            CM *= Math.sqrt(2);
        }

        double tickCM = 1120 / 26.928;
        tickCM *= (100f/141f);
        long ticks = Math.round(tickCM * CM);

        return (int) ticks;
    }

    public static double clambAngle(double angle){

        if(angle < -180)
            angle += 360;
        else if(angle > 180)
            angle -= 360;

        return angle;
    }

}
