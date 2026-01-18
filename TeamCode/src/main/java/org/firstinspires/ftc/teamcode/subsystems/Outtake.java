package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake {
    private final DcMotorEx OuttakeTopMotor;
    private final DcMotorEx OuttakeBottomMotor;
    private final double short_pow = .6055;
    private final double long_pow = .67;

    public Outtake(HardwareMap hardwareMap){
        OuttakeTopMotor = hardwareMap.get(DcMotorEx.class,"OuttakeTopMotor");
        OuttakeBottomMotor = hardwareMap.get(DcMotorEx.class,"OuttakeBottomMotor");
    }
    public void runMotor(double top_pow,double bottom_pow){
        OuttakeTopMotor.setPower(top_pow);
        OuttakeBottomMotor.setPower(bottom_pow);
    }
    public void runMotor(double p){
        OuttakeTopMotor.setPower(p);
        OuttakeBottomMotor.setPower(p);
    }
    public void stopMotors(){
        runMotor(0,0);
    }
    /*public double getBottomRPM(){
        //does not work
        return (OuttakeBottomMotor.getVelocity()/28)*60;
    }
    public double getTopRPM(){
        //does not work
        return (OuttakeTopMotor.getVelocity()/28)*60;
    }*/
    public void short_outtake(){
        runMotor(short_pow-0.07,short_pow);
    }
    public void long_outtake(){
        runMotor(long_pow-0.07,long_pow);
    }
    public void intake(){
        runMotor(-1);
    }
}
