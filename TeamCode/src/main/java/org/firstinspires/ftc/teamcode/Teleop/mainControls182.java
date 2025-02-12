package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.mainMethods;

@TeleOp(name = "(182) Main Controls")
public class mainControls182 extends LinearOpMode {
    @Override
    public void runOpMode(){
        // Initialize
        ElapsedTime runtime = new ElapsedTime();
        DcMotor frontLeft;
        DcMotor frontRight;
        DcMotor backLeft;
        DcMotor backRight;
        DcMotor leftArm;
        DcMotor rightArm;
        Servo gripper;
        CRServo wrist;

        double drive, turn, strafe;
        double frPower, flPower, brPower, blPower;

        double gripperClosedPosition = 1.0;
        double gripperOpenPosition = 0;

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        leftArm = hardwareMap.get(DcMotor.class, "leftArm");
        rightArm= hardwareMap.get(DcMotor.class, "rightArm");
        gripper = hardwareMap.get(Servo.class, "gripper");
        wrist = hardwareMap.get(CRServo.class, "wrist");


        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        leftArm.setDirection(DcMotor.Direction.REVERSE);

        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        telemetry.addData("Status", "Initialized");

        waitForStart();
        // Start
        while (opModeIsActive()) {
            // Movement
//            flPower = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
//            frPower = gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
//            blPower = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
//            brPower = gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
//

            drive = gamepad1.left_stick_y*-1;
            turn = gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;

            flPower = drive + turn + strafe;
            frPower = drive - turn - strafe;
            blPower = drive + turn - strafe;
            brPower = drive - turn + strafe;

            frontLeft.setPower(flPower);
            frontRight.setPower(frPower);
            backLeft.setPower(blPower);
            backRight.setPower(brPower);



            //ARM & WRIST
            rightArm.setPower(gamepad2.left_stick_y);
            leftArm.setPower(gamepad2.left_stick_y);
            wrist.setPower(gamepad2.right_stick_y);

            // HANGING BUTTON
            if (gamepad2.triangle){
                rightArm.setPower(rightArm.getPower());
                leftArm.setPower(leftArm.getPower());
                sleep(99999999);
            }

            //GRIPPER
            // Checks to see if has been pressed before and stops if it has.
            if (gamepad2.right_bumper) {
                gripper.setPosition(gripperClosedPosition);
            } else {
                gripper.setPosition(gripperOpenPosition);
            }


            telemetry.addData("Position", frontLeft.getCurrentPosition());
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.update();

        }
    }

}