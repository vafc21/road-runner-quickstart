package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Handoff {
    private final CRServo top;
    private final CRServo bottom;
    private final double pow = 0.5;
    public Handoff(HardwareMap hardwareMap){
        top = hardwareMap.get(CRServo.class, "TopHandoff");
        bottom = hardwareMap.get(CRServo.class, "BottomHandoff");
    }
    public void runMotor(double pow){
        top.setPower(pow);
        bottom.setPower(pow);
    }
    public void stopMotors(){
        runMotor(0);
    }
    public void store(boolean t){
        if (t) {
            top.setPower(pow);
            bottom.setPower(pow);
        }
        stopMotors();
    }
    public void handoff(boolean t){
        if(t){
            top.setPower(-pow);
            bottom.setPower(pow);
        }
        stopMotors();
    }

}
