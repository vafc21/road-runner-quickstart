package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.core.math.MathUtils;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.util.PIDController;

@TeleOp(name="TeleopOp_Main")
public class MainTeleOpMode extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor FRMotor = null;
//    private DcMotor FLMotor = null;
//    private DcMotor BRMotor;
//    private DcMotor BLMotor;
    private DcMotorEx Pivot;
    private CRServo Intake;
    private DcMotor Extension;
    private CRServo dispenser;
    private double kp = 0.007;
    private int pivotSetpoint = 0;
    private int extensionSetpoint = 0;
    private int intakeSetpoint = 0;
    private Pose2d StartPose = new Pose2d(0, 0, 0);
    PIDController pivotPID = new PIDController(0.007);
    PIDController extensionPID = new PIDController(0.007);
    PIDController intakePID = new PIDController(0.007);
    private final ElapsedTime armHomeTimer = new ElapsedTime();
    private boolean armHoming = false;
    private boolean armHomingReset = true;

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
        Pivot = hardwareMap.get(DcMotorEx.class, "Pivot");
        Extension = hardwareMap.get(DcMotor.class, "exten");
        dispenser = hardwareMap.get(CRServo.class, "dispenser");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        Pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Pivot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        FRMotor.setDirection(DcMotor.Direction.REVERSE);
//        BRMotor.setDirection(DcMotor.Direction.FORWARD);
        Pivot.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double BLpower;
            double FLpower;
            double FRpower;
            double BRpower;
            double DConstant=1;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
//            double rotateSpeed = 0.8;
            float pivotSpeed = 0.8f;
            float extensionSpeed = 0.8f;
            double rotateSpeed = 0.8;
            double turn = -gamepad2.left_stick_x;
            double drive  =  gamepad2.left_stick_y;
            double rotate = -gamepad2.right_stick_x * rotateSpeed;
            double speedForDrive = 0.9;
            boolean tubomodeForPlayer2 = gamepad2.y;
            boolean tubomodeForPlayer1 = gamepad1.y;
            boolean tubomodeForPlayer1_On = false;
            boolean tubomodeForPlayer2_On = false;
            int dispenserIsIntakingToggle = 0;
            boolean dispenserIsIntaking = false;
            boolean isGamepad2LeftStickPressed = gamepad2.left_stick_button;
            float G2RT = gamepad2.right_trigger;
            float G2LT = gamepad2.left_trigger;

            int extensionHome = 650;
            int home = 132;
            int pos1 = 400;
            float extensionOut = gamepad1.right_stick_y * extensionSpeed;
            float extensionIn = gamepad1.right_stick_y * extensionSpeed;
            //boolean goToPos1=gamepad1.b;
            //boolean pivotUp = gamepad1.dpad_up;
            //boolean pivotDown = gamepad1.dpad_down;
            boolean goToHome=gamepad1.x;
            boolean isGamepad1LeftStickPressed = gamepad1.left_stick_button;
            boolean isGamepad1RightStickPressed = gamepad1.right_stick_button;

            float pivotUp=-gamepad1.left_stick_y * pivotSpeed;
            float pivotDown= -gamepad1.left_stick_y * pivotSpeed;
            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;
            if (tubomodeForPlayer2) {
                speedForDrive = 1.0;
                rotateSpeed = 1.0;
            }else {
                if (isGamepad2LeftStickPressed) {
                    speedForDrive = 1.0;
                    rotateSpeed = 1.0;
                } else {
                    speedForDrive = 0.8;
                    rotateSpeed = 0.8;
                }

            }
            if (G2RT > 0.01){
                rotate += G2RT*0.5;

            }else if (G2LT > 0.01){
                rotate -= G2LT*0.5;
            }
            BLpower = DConstant*Range.clip(-drive - turn + rotate, -1.0, 1.0) ;
            FLpower = DConstant*Range.clip(drive - turn - rotate, -1.0, 1.0) ;
            FRpower = DConstant*Range.clip(drive + turn + rotate, -1.0, 1.0) ;
            BRpower = DConstant*Range.clip(-drive + turn - rotate, -1.0, 1.0) ;
//
//            // Send calculated power to wheels
//            FLMotor.setPower(FLpower * speedForDrive);
//            BLMotor.setPower(BLpower * speedForDrive);
//            BRMotor.setPower(BRpower * speedForDrive);
//            FRMotor.setPower(FRpower * speedForDrive);
            Vector2d translationalVelocity = new Vector2d(DConstant * drive, DConstant * turn);
            double rotationalVelocity = DConstant * rotate;

            PoseVelocity2d velocity = new PoseVelocity2d(translationalVelocity, rotationalVelocity);

            Drive.setDrivePowers(velocity);





            //This is for moving the pivot
            if (goToHome && armHoming) {
                armHoming = false;
                pivotSetpoint = Pivot.getCurrentPosition();
                extensionSetpoint = Extension.getCurrentPosition();
            } else if (goToHome || armHoming){
                armHoming = true;
                extensionSetpoint = extensionHome;
                if(Extension.getCurrentPosition() <= extensionHome + 10) {
                    if (armHomeTimer.seconds() > 1.0) {
                        pivotSetpoint = home;
                    } else if (armHomingReset) {
                        armHomeTimer.reset();
                        armHomingReset = false;
                    }

                    if (Pivot.getCurrentPosition() <= home + 10) {
                        armHoming = false;
                        armHomingReset = true;
                    }
                }
            }
//            else if (goToPos1) {
//                pivotSetpoint = pos1;
//            }
            if (tubomodeForPlayer1 || tubomodeForPlayer1_On) {
                tubomodeForPlayer1_On = true;
                pivotSpeed = 1.0f;
                extensionSpeed = 1.0f;
            } else {
                tubomodeForPlayer1_On = false;
                if (isGamepad1LeftStickPressed) {
                    pivotSpeed = 1.0f;
                } else {
                    pivotSpeed = 0.8f;
                }

                if (isGamepad1RightStickPressed) {
                    extensionSpeed = 1.0f;
                } else {
                    extensionSpeed = 0.8f;
                }
            }
            if (pivotUp > 0.01) {
                pivotSetpoint += pivotUp * 20;

            } else if (pivotDown < -0.01) {
                pivotSetpoint -= pivotDown * -20;
            }
            if (extensionOut > 0.01){
                extensionSetpoint += extensionOut*20;

            } else if (extensionIn < -0.01){
                extensionSetpoint -= extensionIn*-20;

            }

            pivotSetpoint = MathUtils.clamp(pivotSetpoint, 144, 2330);
            //extensionSetpoint = MathUtils.clamp(extensionSetpoint,);
            double outputE = extensionPID.calculate(Extension.getCurrentPosition(),extensionSetpoint);
            Extension.setPower(outputE + 0.01);
            double output = pivotPID.calculate(Pivot.getCurrentPosition(),pivotSetpoint);
            output = MathUtils.clamp(output, -0.25, 1.0);
            Pivot.setPower(output + 0.01);
//            if (gamepad1.left_bumper) {
//                if (dispenserIsIntakingToggle == 1) {
//                    dispenserIsIntaking = true;
//                    dispenser.setPower(-1);
//                    dispenserIsIntakingToggle = 0;
//                }else if (dispenserIsIntakingToggle == 0) {
//                    dispenserIsIntaking = false;
//                    dispenser.setPower(0);
//                    dispenserIsIntakingToggle = 1;
//                }
//                dispenserIsIntaking = false;
//
//            }
            if (gamepad1.left_bumper) {
                dispenser.setPower(-1);
            }
            else if (gamepad1.right_bumper){
                dispenser.setPower(1);

            }
            else{
                dispenser.setPower(0);
            }



            // Show the elapsed game time and wheel power.
            //telemetry.addData("Left Trigger", -intake);
            //telemetry.addData("Right Trigger", dispense);
            telemetry.addData("Turbo mode for player 1", tubomodeForPlayer1);
            telemetry.addData("Turbo mode for player 2", tubomodeForPlayer2);
            telemetry.addData("Arm Langth", Extension.getCurrentPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Pivot Angle", Pivot.getCurrentPosition());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}