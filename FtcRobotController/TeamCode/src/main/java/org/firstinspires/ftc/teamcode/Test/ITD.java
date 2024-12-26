package org.firstinspires.ftc.teamcode.Test;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
@TeleOp
public class ITD extends OpMode {
    public Servo servo2, servo3, servo4;
    public CRServo servo0, servo1;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    @Override
    public void init() {
        servo2 = hardwareMap.get(Servo.class, "servo");
        servo3 = hardwareMap.get(Servo.class, "servo3");
        servo4 = hardwareMap.get(Servo.class, "servo4");
        servo0 = hardwareMap.get(CRServo.class, "servo0");
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        motor1 = hardwareMap.get(DcMotor.class, "slideup1");
        motor2 = hardwareMap.get(DcMotor.class, "slideup2");
        motor3 = hardwareMap.get(DcMotor.class, "slidedown3");
    }
    @Override
    public void loop() {
        if (gamepad1.a) {
            servo2.setPosition(0.29);
        } else if (gamepad1.b) {
            servo2.setPosition(0.38);
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

        if (gamepad1.x) {
            servo0.setPower(1);
            servo1.setPower(-1);
        } else if (gamepad1.y) {
            servo0.setPower(-1);
            servo1.setPower(1);
        } else {
            servo0.setPower(0);
            servo1.setPower(0);
        }

        // Slides up down
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

        // slide down
        if (gamepad1.left_bumper) {
            motor3.setPower(1.0);
        } else if (gamepad1.right_bumper) {
            motor3.setPower(-1.0);
        } else {
            motor3.setPower(0.0);
        }
// Display telemetry
        telemetry.addData("Motor1 Position", motor1.getCurrentPosition());
        telemetry.addData("Motor2 Position", motor2.getCurrentPosition());
        telemetry.addData("Motor3 Position", motor3.getCurrentPosition());
        telemetry.update();
    }
}
