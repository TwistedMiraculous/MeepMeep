package org.firstinspires.ftc.teamcode.Main;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class FinalTeleOP extends OpMode {
    public Servo servo2, servo3, servo4;
    public CRServo servo0, servo1;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private BNO055IMU imu;
    private Orientation angles;
    private Rev2mDistanceSensor distanceSensor;
    private DigitalChannel led;
    private DigitalChannel touchSensor;
    private DigitalChannel touchSensor0;

    @Override
    public void init() {
        servo2 = hardwareMap.get(Servo.class, "servo");
        servo3 = hardwareMap.get(Servo.class, "servo3");
        servo4 = hardwareMap.get(Servo.class, "servo4");
        servo0 = hardwareMap.get(CRServo.class, "servo0");
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");
        touchSensor0 = hardwareMap.get(DigitalChannel.class, "touchSensor0");

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Initialize the distance sensor from the hardware map
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");

        // Initialize the LED from the hardware map
        led = hardwareMap.get(DigitalChannel.class, "led");

        // Set the LED as an output
        led.setMode(DigitalChannel.Mode.OUTPUT);

        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");

        // Reset encoders and initialize motors
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set touch sensor mode
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        // Reset encoders and initialize motors
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set touch sensor mode
        touchSensor0.setMode(DigitalChannel.Mode.INPUT);

    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            servo2.setPosition(0.305);
        } else if (gamepad1.b) {
            servo2.setPosition(0.41);
        }

        if (gamepad1.dpad_up) {
            servo3.setPosition(0.6);
        } else if (gamepad1.dpad_down) {
            servo3.setPosition(0.051);
        }

        if (gamepad1.dpad_left) {
            servo4.setPosition(0);
        } else if (gamepad1.dpad_right) {
            servo4.setPosition(0.4);
        }

        if (gamepad1.y) {
            servo0.setPower(1);
            servo1.setPower(-1);
        } else if (gamepad1.x) {
            servo0.setPower(-1);
            servo1.setPower(1);
        } else {
            servo0.setPower(0);
            servo1.setPower(0);
        }
/*
        if (gamepad1.left_trigger > 0) {
            motor1.setPower(1.0);
            motor2.setPower(-1.0);
        } else if (gamepad1.right_trigger > 0) {
            motor1.setPower(-1.0);
            motor2.setPower(1.0);
        } else {
            motor1.setPower(0.0);
            motor2.setPower(0.0);
        }

        if (gamepad1.left_bumper) {
            motor3.setPower(1.0);
        } else if (gamepad1.right_bumper) {
            motor3.setPower(-1.0);
        } else {
            motor3.setPower(0.0);
        }
        */

        // Get the gamepad inputs
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;



        // Get the current orientation from the IMU
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double botHeading = -angles.firstAngle;

        // Rotate the input movement direction to account for the robot's current orientation
        double rotX = x * Math.cos(Math.toRadians(botHeading)) - y * Math.sin(Math.toRadians(botHeading));
        double rotY = x * Math.sin(Math.toRadians(botHeading)) + y * Math.cos(Math.toRadians(botHeading));

        // Calculate the power for each wheel
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // Set the power to the motors
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);


        // Read the distance value in cm
        double distance = distanceSensor.getDistance(DistanceUnit.CM);

        // Check if the distance is below 4 cm
        if (distance > 4) {
            // Turn on the LED
            led.setState(true);
        } else {
            // Turn off the LED
            led.setState(false);
        }

        boolean touchSensorPressed = !touchSensor.getState();

        if (touchSensorPressed) { // Check if touch sensor is pressed
            // Reset encoders once
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Set target positions
            motor1.setTargetPosition(-30);
            motor2.setTargetPosition(30);

            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Apply power to reach the target positions
            motor1.setPower(-0.5); // Adjust as needed
            motor2.setPower(0.5);

        } else if (gamepad1.right_trigger > 0.1) {

            // Set target positions
            motor1.setTargetPosition(-2750);
            motor2.setTargetPosition(2750);


            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Move slides up
            motor1.setPower(-1);
            motor2.setPower(1);

        } else if (gamepad1.left_trigger > 0.1) {
            // Move slides down

            // Set target positions
            motor1.setTargetPosition(10);
            motor2.setTargetPosition(-10);

            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Move slides down
            motor1.setPower(1);
            motor2.setPower(-1);
        } else if (gamepad1.left_stick_button) {
            // Move slides down

            // Set target positions
            motor1.setTargetPosition(10);
            motor2.setTargetPosition(-10);

            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Move slides down
            motor1.setPower(1);
            motor2.setPower(-1);

            servo4.setPosition(0.4);
        } else if (gamepad1.right_stick_button) {

            // Set target positions
            motor1.setTargetPosition(-1800);
            motor2.setTargetPosition(1800);

            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Move slides up
            motor1.setPower(-1);
            motor2.setPower(1);
        }


        boolean touchSensor0Pressed = !touchSensor0.getState();

        if (touchSensor0Pressed) { // Check if touch sensor is pressed
            // Reset encoders once
            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor3.setTargetPosition(30);
            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set target positions


            // Apply power to reach the target positions
            motor3.setPower(.5); // Adjust as needed

        } else if (gamepad1.left_bumper) {
            // Set target positions
            motor3.setTargetPosition(3900);
            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Move slides up
            motor3.setPower(1);

        } else if (gamepad1.right_bumper) {
            // Move slides down
            // Set target positions
            motor3.setTargetPosition(-20);
            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Move slides down
            motor3.setPower(1);
        }



        // Display the distance value on telemetry
        telemetry.addData("Distance (cm)", distance);
        //telemetry.addData("Distance detected: ", distanceSensor.)
        // Display telemetry
        telemetry.addData("Motor1 Position", motor1.getCurrentPosition());
        telemetry.addData("Motor2 Position", motor2.getCurrentPosition());
        telemetry.addData("Motor3 Position", motor3.getCurrentPosition());

        telemetry.addData("Motor1 Target Position", motor1.getTargetPosition());
        telemetry.addData("Motor2  Target Position", motor2.getTargetPosition());
        //telemetry.addData("Motor1 Target", targetPosition1);
        //telemetry.addData("Motor2 Target", targetPosition2);
        telemetry.addData("Touch Sensor Pressed", touchSensorPressed);
        telemetry.update();
    }
}