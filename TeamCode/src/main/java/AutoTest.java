import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Handoff;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

@Autonomous(name = "AutoTest")
public class AutoTest extends LinearOpMode {
    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
    Intake intake = new Intake(hardwareMap);
    Outtake outtake = new Outtake(hardwareMap);
    Handoff handoff = new Handoff(hardwareMap);
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .stopAndAdd(new IntakeAction(true))
                        .lineToX(64)
                        .stopAndAdd(new IntakeAction(false))
                        .lineToX(0)
                        .stopAndAdd(new OutakeAction(true))
                        .lineToX(64)
                        .stopAndAdd(new OutakeAction(false))
                        .lineToX(0)
                        .stopAndAdd(new HandoffAction(true))
                        .lineToX(64)
                        .stopAndAdd(new HandoffAction(false))
                        .lineToX(0)
                        .stopAndAdd(new StoreAction(true))
                        .lineToX(64)
                        .stopAndAdd(new StoreAction(false))
                        .lineToX(0)
                        .stopAndAdd(new StopHandoff())
                        .lineToX(64)
                        .lineToX(0)
                        .build());
    }
    public class IntakeAction implements Action {
        boolean b;
        public IntakeAction(boolean b){
            this.b = b;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.intake(b,!b);
            return false;
        }
    }
    public class OutakeAction implements Action {
        boolean b;
        public OutakeAction(boolean b){
            this.b = b;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            outtake.outtake(b);
            return false;
        }
    }
    public class HandoffAction implements Action {
        boolean b;
        public HandoffAction(boolean b){
            this.b = b;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            handoff.handoff(b);
            return false;
        }
    }
    public class StoreAction implements Action {
        boolean b;
        public StoreAction(boolean b){
            this.b = b;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            handoff.handoff(b);
            return false;
        }
    }
    public class StopHandoff implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            handoff.stopMotors();
            return false;
        }
    }
}
