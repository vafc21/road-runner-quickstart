package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake {
    private final DcMotor OuttakeTopMotor;
    private final DcMotor OuttakeBottomMotor;
    public Outtake(HardwareMap hardwareMap){
        OuttakeTopMotor = hardwareMap.get(DcMotor.class,"OuttakeTopMotor");
        OuttakeBottomMotor = hardwareMap.get(DcMotor.class,"OuttakeBottomMotor");
    }
    public void runMotor(double top_pow,double bottom_pow){
        OuttakeTopMotor.setPower(top_pow);
        OuttakeBottomMotor.setPower(bottom_pow);
    }
    public void stopMotors(){
        runMotor(0,0);
    }
    public void outtake(boolean p){
        if (p){
            double topPow = 1;
            OuttakeTopMotor.setPower(topPow);
            double bottomPow = 1;
            OuttakeBottomMotor.setPower(bottomPow);
        }
        else {
            OuttakeTopMotor.setPower(0);
            OuttakeBottomMotor.setPower(0);
        }
    }
}
