package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.mainMethods;
@TeleOp(name = "(004) Main Controls")
public class mainControls004 extends LinearOpMode {
    @Override
    public void runOpMode(){
        DcMotor frontLeft;
        DcMotor frontRight;
        DcMotor backLeft;
        DcMotor backRight;
        CRServo leftArm;
        CRServo rightArm;
        Servo gripper;
        CRServo rightWrist;
        CRServo leftWrist;
        DcMotor leftWinch;
        CRServo rightWinch;

        double gripperClosedPosition = 1.0;
        double gripperOpenPosition = 0;
        double drive, turn, strafe;
        double frPower, flPower, brPower, blPower;

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        leftArm = hardwareMap.get(CRServo.class, "leftArm");
        rightArm= hardwareMap.get(CRServo.class, "rightArm");
        gripper = hardwareMap.get(Servo.class, "gripper");
        rightWrist = hardwareMap.get(CRServo.class, "rightWrist");
        leftWrist = hardwareMap.get(CRServo.class, "leftWrist");
        rightWinch = hardwareMap.get(CRServo.class, "rightWinch");
        leftWinch = hardwareMap.get(DcMotor.class, "leftWinch");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        rightArm.setDirection(DcMotor.Direction.REVERSE);
        leftWrist.setDirection(DcMotor.Direction.REVERSE);
        leftWinch.setDirection(DcMotor.Direction.REVERSE);


        leftWinch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        // Start
        while (opModeIsActive()) {
            // Movement
            drive = gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;

            flPower = drive - turn - strafe;
            frPower = drive + turn + strafe;
            blPower = drive - turn + strafe;
            brPower = drive + turn - strafe;

            if (Math.abs(flPower) > 1) {
                flPower = flPower/Math.abs(flPower);
            }
            if (Math.abs(frPower) > 1) {
                frPower = frPower/Math.abs(frPower);
            }
            if (Math.abs(blPower) > 1) {
                blPower = blPower/Math.abs(blPower);
            }
            if (Math.abs(brPower) > 1) {
                brPower = brPower/Math.abs(brPower);
            }

            frontLeft.setPower(flPower);
            frontRight.setPower(blPower);
            backLeft.setPower(frPower);
            backRight.setPower(brPower);

            // Arm control
            rightArm.setPower(gamepad2.left_stick_y);
            leftArm.setPower(gamepad2.left_stick_y);
            rightWrist.setPower(gamepad2.right_stick_y);
            leftWrist.setPower(gamepad2.right_stick_y);
            rightWinch.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            leftWinch.setPower(gamepad1.right_trigger - gamepad1.left_trigger);


            // HANGING BUTTON
            if (gamepad1.triangle){
                sleep(99999999);
            }

            //GRIPPER
            if (gamepad2.right_bumper) {
                gripper.setPosition(gripperClosedPosition);
            } else {
                gripper.setPosition(gripperOpenPosition);
            }
        }
    }
}