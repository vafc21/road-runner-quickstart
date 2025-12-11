package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final DcMotor IntakeMotor;
    public Intake(HardwareMap hardwareMap) {
        IntakeMotor = hardwareMap.get(DcMotor.class,"IntakeMotor");
    }
    public void runMotor(double pow){
        IntakeMotor.setPower(pow);
    }
    public void stopMotor(){
        runMotor(0);
    }
    public void intake(boolean p1,boolean p2){
        double pow = 1;
        if (p1){
            runMotor(pow);
        } else if (p2) {
            runMotor(-pow);
        } else {
            stopMotor();
        }
    }
}
