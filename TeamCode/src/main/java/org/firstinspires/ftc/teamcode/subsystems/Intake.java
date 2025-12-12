package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final DcMotor IntakeMotor;
    private boolean hasStarted = false;
    private boolean hasStopped = false;
    private final double pow = 1;
    private boolean toggleV = false;

    public Intake(HardwareMap hardwareMap) {
        IntakeMotor = hardwareMap.get(DcMotor.class,"IntakeMotor");
        IntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    //public boolean getHasStopped(){ return hasStopped;}
    public void runMotor(double pow){
        if(!hasStopped) {
            IntakeMotor.setPower(pow);
        }
    }
    public void stopMotor(){
        runMotor(0);
        hasStopped = true;

    }
    public void intake(boolean p1,boolean p2){

        if (p1){
            hasStopped = false;
            runMotor(pow);
        } else if (p2) {
            hasStopped = false;
            runMotor(-pow);
        } else {
            stopMotor();
        }
    }
    public void toggle(boolean t){
        if (t){
            if (toggleV){
                intake(true,false);
                toggleV = false;
            } else {
                intake(false,true);
                toggleV = true;
            }
        }
    }
    public void takeInToggle(int i){

        if (i==2){
            hasStopped = false;
            runMotor(pow);
            hasStarted = true;
        } else if (i==1){
            if (hasStarted) {
                hasStopped = false;
                runMotor(-pow);
            }
        }
    }
}
