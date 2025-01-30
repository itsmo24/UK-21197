package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.mainMethods;

@TeleOp(name = "Main Controls")
public class mainControls extends LinearOpMode {
    @Override
    public void runOpMode(){
        // Initialize
        mainMethods move;
        ElapsedTime runtime = new ElapsedTime();
        DcMotor frontLeft;
        DcMotor frontRight;
        DcMotor backLeft;
        CRServo backRight;
        CRServo leftArm;
        CRServo rightArm;
        Servo gripper;
        CRServo wrist;

        double gripperClosedPosition = 1.0;
        double gripperOpenPosition = 0;
        int armHomePosition = 0;

        move = new mainMethods(hardwareMap);
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(CRServo.class, "backRight");
        leftArm = hardwareMap.get(CRServo.class, "leftArm");
        rightArm= hardwareMap.get(CRServo.class, "rightArm");
        gripper = hardwareMap.get(Servo.class, "gripper");
        wrist = hardwareMap.get(CRServo.class, "wrist");


        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(CRServo.Direction.REVERSE);
        leftArm.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Initialized");

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
            if (gamepad2.dpad_up){
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