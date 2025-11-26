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
    public void outtake(boolean p){
        if (p){
            double topPow = .5;
            OuttakeTopMotor.setPower(topPow);
            double bottomPow = .5;
            OuttakeBottomMotor.setPower(bottomPow);
        }
    }
}
