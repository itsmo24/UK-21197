package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.mainMethods;

@TeleOp(name = "(004) Main Controls")
public class mainControls004 extends LinearOpMode {

    @Override
    public void runOpMode(){
        // Initialize

        ElapsedTime runtime = new ElapsedTime();
        PIDFCoefficients pidfCoefficients;
        DcMotorEx frontLeft;
        DcMotorEx frontRight;
        DcMotorEx backLeft;
        DcMotorEx backRight;
        DcMotorEx leftArm;
        DcMotorEx rightArm;
        Servo gripper;
        CRServo rightWrist;
        CRServo leftWrist;
        DcMotorEx leftWinch;
        DcMotorEx rightWinch;



        double gripperClosedPosition = 1.0;
        double gripperOpenPosition = 0;
        double FL;
        double FLMax = 0.0;
        double FR;
        double FRMax = 0.0;
        double BL;
        double BLMax = 0.0;
        double BR;
        double BRMax = 0.0;


        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        leftArm = hardwareMap.get(DcMotorEx.class, "leftArm");
        rightArm= hardwareMap.get(DcMotorEx.class, "rightArm");
        gripper = hardwareMap.get(Servo.class, "gripper");
        rightWrist = hardwareMap.get(CRServo.class, "rightWrist");
        leftWrist = hardwareMap.get(CRServo.class, "leftWrist");
        rightWinch = hardwareMap.get(DcMotorEx.class, "rightWinch");
        leftWinch = hardwareMap.get(DcMotorEx.class, "leftWinch");



        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        leftArm.setDirection(DcMotor.Direction.REVERSE);
        leftWrist.setDirection(CRServo.Direction.REVERSE);
        leftWinch.setDirection(DcMotor.Direction.REVERSE);



        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftWinch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWinch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /*frontLeft.setVelocityPIDFCoefficients( 1.15, 0.115, 0,11.5);
        frontRight.setVelocityPIDFCoefficients(3.45,0.345,0,34.5);
        backLeft.setVelocityPIDFCoefficients(1.2,0.12,0,12);
        backRight.setVelocityPIDFCoefficients(1.13,0.113,0,11.3);*/
        //front left max velocity is 2860
        //back left max velocity is 2720

        //front right max velocity is 980
        //back right max velocity is 2900
        //32767/FLMax
        //https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotorEx.html#getVelocity-org.firstinspires.ftc.robotcore.external.navigation.AngleUnit-
        //https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit?tab=t.0#heading=h.h2mitzlvr4py
        //https://github.com/NoahBres/VelocityPIDTuningTutorial/blob/master/README.md



        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        // Start
        while (opModeIsActive()) {
            // Movement





            double topLeftSpeed = -gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            double bottomLeftSpeed = -gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
            double topRightSpeed = -gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
            double bottomRightSpeed = -gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;

            frontLeft.setVelocity(3000*topLeftSpeed);
            frontRight.setVelocity(3000*topRightSpeed);
            backLeft.setVelocity(3000*bottomLeftSpeed);
            backRight.setVelocity(3000*bottomRightSpeed);
            //backLeft.set
            FL = (backLeft.getVelocity());
            FR = (backLeft.getVelocity());
            BR = (backLeft.getVelocity());
            BL = (backLeft.getVelocity());
            if (FL > FLMax) {
                FLMax = FL;
            }
            if (FR > FRMax) {
                FRMax = FR;
            }
            if (BR > BRMax) {
                BRMax = BR;
            }
            if (BL > BLMax) {
                BLMax = BL;
            }
            telemetry.addData("FL:", FL);
            telemetry.addData("FLMax:", FLMax);
            telemetry.addData("FR:", FR);
            telemetry.addData("FRMax:", FRMax);
            telemetry.addData("BL:", BL);
            telemetry.addData("BLMax:", BLMax);
            telemetry.addData("BR:", BR);
            telemetry.addData("BRMax:", BRMax);

            telemetry.update();

            //ARM & WRIST
            rightArm.setPower(-gamepad2.left_stick_y);
            leftArm.setPower(-gamepad2.left_stick_y);
            rightWrist.setPower(gamepad2.right_stick_y);
            leftWrist.setPower(gamepad2.right_stick_y);
            rightWinch.setPower(gamepad1.right_trigger);
            leftWinch.setPower(gamepad1.right_trigger);


            // HANGING BUTTON
            if (gamepad1.triangle){
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



            //telemetry.addData("Status", "Run Time: " + runtime);
            //telemetry.update();

        }
    }

}