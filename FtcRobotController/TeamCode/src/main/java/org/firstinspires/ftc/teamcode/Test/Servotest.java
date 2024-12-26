package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Servotest extends LinearOpMode {
    private CRServo servo, servo2;

    @Override
    public void runOpMode() {
        // Initialize the servo
        servo = hardwareMap.get(CRServo.class, "servo");
        servo.setPower(0); // Default position

        servo2 = hardwareMap.get(CRServo.class, "servo2");
        servo2.setPower(0);

        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();
        waitForStart();

        // Main control loop
        while (opModeIsActive()) {
            turn();



            // Display the current servo position
            telemetry.addData("Servo Position", servo2.getPower());
            telemetry.addData("Servo Position", servo.getPower());
            telemetry.update();
        }
    }

    public void turn() {
        // Check button presses and set servo position
        if (gamepad1.a) {
            servo.setPower(0.3);
            servo2.setPower(-0.3);
        } else if (gamepad1.b) {
            servo.setPower(-0.3);
            servo2.setPower(0.3);
        } else {
            servo.setPower(0); // Move servo to position 0.04 (slightly extended)
            servo2.setPower(0);
            sleep(200); // Brief delay for debouncing
        }
    }



}
