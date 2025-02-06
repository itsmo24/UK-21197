package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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
        DcMotor frontLeft;
        DcMotor frontRight;
        DcMotor backLeft;
        DcMotor backRight;
        CRServo leftArm;
        CRServo rightArm;
        Servo gripper;
        CRServo wrist;

        double gripperClosedPosition = 1.0;
        double gripperOpenPosition = 0;


        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        leftArm = hardwareMap.get(CRServo.class, "leftArm");
        rightArm= hardwareMap.get(CRServo.class, "rightArm");
        gripper = hardwareMap.get(Servo.class, "gripper");
        wrist = hardwareMap.get(CRServo.class, "wrist");


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(CRServo.Direction.REVERSE);
        leftArm.setDirection(DcMotor.Direction.REVERSE);



        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        // Start
        while (opModeIsActive()) {
            // Movement
            double topLeftPower = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            double bottomLeftPower = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
            double topRightPower = gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
            double bottomRightPower = gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;

            frontLeft.setPower(topLeftPower);
            frontRight.setPower(topRightPower);
            backLeft.setPower(bottomLeftPower);
            backRight.setPower(bottomRightPower);




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



            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.update();

        }
    }

}