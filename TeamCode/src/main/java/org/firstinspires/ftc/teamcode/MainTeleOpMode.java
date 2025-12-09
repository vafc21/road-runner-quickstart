package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Handoff;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

@TeleOp(name="TeleOp_Main")
public class MainTeleOpMode extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //    private DcMotor FRMotor = null;
//    private DcMotor FLMotor = null;
//    private DcMotor BRMotor;
//    private DcMotor BLMotor;
    private final double kp = 0.007;

    private Pose2d StartPose = new Pose2d(0, 0, 0);


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
//        FRMotor  = hardwareMap.get(DcMotor.class, "FrontRightMotor");
//        FLMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
//        BLMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
//        BRMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");.
        //Intake = hardwareMap.get(CRServo.class, "Intake");
        MecanumDrive Drive = new MecanumDrive(hardwareMap, StartPose);
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        Handoff handoff = new Handoff(hardwareMap);
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips


//        FRMotor.setDirection(DcMotor.Direction.REVERSE);
//        BRMotor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double DConstant=1;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
//            double rotateSpeed = 0.8;
            double rotateSpeed = 0.8;
            double turn = gamepad1.left_stick_x;
            double drive  =  gamepad1.left_stick_y;
            double rotate = gamepad1.right_stick_x * rotateSpeed;
            double speedForDrive = 0.9;
            boolean isGamepad2LeftStickPressed = gamepad2.left_stick_button;
            float G2RT = gamepad2.right_trigger;
            float G2LT = gamepad2.left_trigger;



            boolean isGamepad1LeftStickPressed = gamepad1.left_stick_button;
            boolean isGamepad1RightStickPressed = gamepad1.right_stick_button;


            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;
            if (gamepad1.left_trigger > 0.1 && gamepad1.right_trigger > 0.1){
                stop();
                outtake.stopMotors();
                intake.stopMotor();
                handoff.stopMotors();
                break;
            } else {
                intake.intake(gamepad1.a, gamepad1.b);
                outtake.outtake(gamepad1.right_bumper);
                handoff.handoff(gamepad1.left_bumper);
                handoff.store(gamepad1.x);
                if (gamepad1.y) handoff.stopMotors();
            }

            /*
            if (G2RT > 0.01){
                rotate += G2RT*0.5;

            }else if (G2LT > 0.01){
                rotate -= G2LT*0.5;
            }*/

//
//            // Send calculated power to wheels
//            FLMotor.setPower(FLpower * speedForDrive);
//            BLMotor.setPower(BLpower * speedForDrive);
//            BRMotor.setPower(BRpower * speedForDrive);
//            FRMotor.setPower(FRpower * speedForDrive);
            Vector2d translationalVelocity = new Vector2d(DConstant * -drive, DConstant * -turn);
            double rotationalVelocity = DConstant * -rotate;

            PoseVelocity2d velocity = new PoseVelocity2d(translationalVelocity, rotationalVelocity);

            Drive.setDrivePowers(velocity);


            // Show the elapsed game time and wheel power.
            telemetry.addData("poseVX",Drive.updatePoseEstimate().linearVel.x);
            telemetry.addData("posVY",Drive.updatePoseEstimate().linearVel.y);
            telemetry.addData("AngV", Drive.updatePoseEstimate().angVel);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}