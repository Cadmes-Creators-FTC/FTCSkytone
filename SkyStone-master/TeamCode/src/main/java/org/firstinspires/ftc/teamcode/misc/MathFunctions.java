package org.firstinspires.ftc.teamcode.misc;

@SuppressWarnings("unused")
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

    public static double clambAngleDegrees(double angle){

        while (angle < -180)
            angle += 360;
        while (angle > 180)
            angle -= 360;

        return angle;
    }

    public static double clambAngleRadians(double angle){

        while (angle < -Math.PI)
            angle += Math.PI * 2;
        while (angle > Math.PI)
            angle -= Math.PI * 2;

        return angle;
    }

}
