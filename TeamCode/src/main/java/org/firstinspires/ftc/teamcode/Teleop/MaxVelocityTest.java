package org.firstinspires.ftc.teamcode.Teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp

public class MaxVelocityTest extends LinearOpMode {

    DcMotorEx frontRight;
    DcMotorEx backRight;
    DcMotorEx backLeft;
    DcMotorEx frontLeft;
    IMU imu;

    double currentVelocity;

    double maxVelocity = 0.0;


    @Override

    public void runOpMode() {

        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");

        waitForStart();


        while (opModeIsActive()) {




            currentVelocity = frontRight.getVelocity();
            frontRight.setPower(1);


            if (currentVelocity > maxVelocity) {

                maxVelocity = currentVelocity;

            }




            telemetry.addData("current velocity", currentVelocity);

            telemetry.addData("maximum velocity", maxVelocity);

            telemetry.update();
            //004
            //front left max velocity is 2860
            //back left max velocity is 2720
            //front right max velocity is 980
            //back right max velocity is 2900
            //182


        }

    }

}