package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake {
    private final DcMotorEx OuttakeTopMotor;
    private final DcMotorEx OuttakeBottomMotor;
    private final double pow = .75;

    public Outtake(HardwareMap hardwareMap){
        OuttakeTopMotor = hardwareMap.get(DcMotorEx.class,"OuttakeTopMotor");
        OuttakeBottomMotor = hardwareMap.get(DcMotorEx.class,"OuttakeBottomMotor");
    }
    public void runMotor(double top_pow,double bottom_pow){
        OuttakeTopMotor.setPower(top_pow);
        OuttakeBottomMotor.setPower(bottom_pow);
    }
    public void runMotor(){
        OuttakeTopMotor.setPower(pow);
        OuttakeBottomMotor.setPower(pow);
    }
    public void stopMotors(){
        runMotor(0,0);
    }
    public double getBottomRPM(){
        return (OuttakeBottomMotor.getVelocity()/28)*60;
    }
    public double getTopRPM(){
        return (OuttakeTopMotor.getVelocity()/28)*60;
    }
    public void outtake(boolean p){
        if (p){
            runMotor();
        }
        else {
            stopMotors();
        }
    }
}
