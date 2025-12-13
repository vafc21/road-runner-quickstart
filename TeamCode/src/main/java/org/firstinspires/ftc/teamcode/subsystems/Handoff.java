package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Handoff {
    private final DcMotor top;
    private final CRServo bottom;
    private boolean toggleV = false;
    private final double pow = 1;
    public Handoff(HardwareMap hardwareMap){
        top = hardwareMap.get(DcMotor.class, "TopHandoff");
        bottom = hardwareMap.get(CRServo.class, "BottomHandoff");
        top.setDirection(REVERSE);

    }
    public void runMotor(double top_pow, double bottom_pow){
        top.setPower(top_pow);
        bottom.setPower(Math.abs(bottom_pow));
    }

    public void setServoPower(double p) {
        bottom.setPower(p);
    }
    public void runMotor(double p){
        top.setPower(p);
        bottom.setPower(Math.abs(p));
    }
    public void stopMotors(){
        runMotor(0);
    }
    public void store(boolean t){
        if (t) {
            runMotor(pow,1);
        }else {
            stopMotors();
        }

    }
    public void storeWait(boolean t){
        if (t){
            ElapsedTime timer = new ElapsedTime();
            timer.startTime();
            while (timer.milliseconds()<3000){
                handoff(true);
            }
            stopMotors();
        }

    }
    public void handoff(boolean t){
        if(t){
            runMotor(-pow,1);

        } else {
            stopMotors();
        }

    }
    public void handoffWait(boolean t){
        if (t){
            ElapsedTime timer = new ElapsedTime();
            timer.startTime();
            while (timer.milliseconds()<300){
                handoff(true);
            }
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
    public void toggleWait(boolean t){
        if (t){
            if (toggleV){
                handoffWait(true);
                toggleV = false;
            } else {
                storeWait(true);
                toggleV = true;
            }
        }
    }
    public int toggleReturn(boolean t){
        if (t){
            if (toggleV){
                handoffWait(true);
                toggleV = false;
                return 1;
            } else {
                storeWait(true);
                toggleV = true;
                return 2;
            }
        }
        return 0;
    }

}
