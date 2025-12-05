package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final DcMotor IntakeMotor;
    public Intake(HardwareMap hardwareMap) {
        IntakeMotor = hardwareMap.get(DcMotor.class,"IntakeMotor");
    }
    public void intake(boolean p1,boolean p2){
        double pow = .5;
        if (p1){
            IntakeMotor.setPower(pow);
        } else if (p2) {
            IntakeMotor.setPower(-pow);
        } else {
            IntakeMotor.setPower(0);
        }
    }
}
