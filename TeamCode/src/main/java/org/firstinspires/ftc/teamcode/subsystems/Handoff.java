package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Handoff {
    private final DcMotor top;
    private final CRServo bottom;
    private final double pow = 1;
    public Handoff(HardwareMap hardwareMap){
        top = hardwareMap.get(DcMotor.class, "TopHandoff");
        bottom = hardwareMap.get(CRServo.class, "BottomHandoff");
    }
    public void runMotor(double top_pow){
        top.setPower(top_pow);
        //bottom.setPower(bottom_pow);
    }
    public void stopMotors(){
        runMotor(0);
    }
    public void store(boolean t){
        if (t) {
            runMotor(pow);
        }else {
            stopMotors();
        }

    }
    public void handoff(boolean t){
        if(t){
            runMotor(pow);

        } else {
            stopMotors();
        }

    }

}
