package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Handoff {
    private final DcMotor top;
    private final CRServo bottom;
    private boolean toggleV = true;
    private final double pow = 1;
    public Handoff(HardwareMap hardwareMap){
        top = hardwareMap.get(DcMotor.class, "TopHandoff");
        bottom = hardwareMap.get(CRServo.class, "BottomHandoff");
        top.setDirection(REVERSE);
    }
    public void runMotor(double top_pow, double bottom_pow){
        top.setPower(top_pow);
        bottom.setPower(bottom_pow);
    }
    public void runMotor(double pow1){
        top.setPower(pow1);
        bottom.setPower(pow1);
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
            runMotor(-pow);

        } else {
            stopMotors();
        }

    }
    public void toggle(boolean t){
        if (t){
            if (toggleV){
                handoff(true);
                toggleV = false;
            } else {
                store(true);
                toggleV = true;
            }
        }
    }

}
