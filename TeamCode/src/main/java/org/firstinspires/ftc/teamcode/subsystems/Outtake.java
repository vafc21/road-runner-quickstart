package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake {
    private final DcMotor OuttakeTopMotor;
    private final DcMotor OuttakeBottomMotor;
    private final double pow = .5;
    public Outtake(HardwareMap hardwareMap){
        OuttakeTopMotor = hardwareMap.get(DcMotor.class,"OuttakeTopMotor");
        OuttakeBottomMotor = hardwareMap.get(DcMotor.class,"OuttakeBottomMotor");
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
    public void outtake(boolean p){
        if (p){
            runMotor();
        }
        else {
            stopMotors();
        }
    }
}
