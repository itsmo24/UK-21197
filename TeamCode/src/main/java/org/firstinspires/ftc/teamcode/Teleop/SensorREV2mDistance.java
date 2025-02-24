package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class SensorREV2mDistance extends LinearOpMode {
    DistanceSensor sensorDistance;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;


    @Override
    public void runOpMode() {
        // Get the distance sensor and motor from hardwareMap
        sensorDistance = hardwareMap.get(DistanceSensor.class, "rangeSensor");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Loop while the Op Mode is running
        waitForStart();
        while (opModeIsActive()) {
            // If the distance in centimeters is less than 10, set the power to 0.3
            if (sensorDistance.getDistance(DistanceUnit.CM) > 10) {
                frontLeft.setPower(0.3);
                frontRight.setPower(0.3);
                backLeft.setPower(0.3);
                backRight.setPower(0.3);
            } else {  // Otherwise, stop the motor
                frontLeft.setPower(0);
            }
        }
    }
}

