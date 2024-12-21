package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class motorTest extends LinearOpMode {
    // Declare motor variables
    private DcMotor motor0;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;

    public void initHardware() {
        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");

        // motor0.setDirection(DcMotor.Direction.REVERSE);
        // motor1.setDirection(DcMotor.Direction.REVERSE);
        // motor2.setDirection(DcMotor.Direction.REVERSE);
        // motor3.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.a) {
                motor0.setPower(-1.0);
                motor1.setPower(-1.0);
            } else if (gamepad1.b) {
                motor0.setPower(1.0);
                motor1.setPower(1.0);
            } else {
                motor0.setPower(0.0);
                motor1.setPower(0.0);
            }

            if (gamepad1.x) {
                motor2.setPower(-1.0);
            } else if (gamepad1.y) {
                motor2.setPower(1.0);
            } else {
                motor2.setPower(0.0);
            }

            if (gamepad1.dpad_up) {
                motor3.setPower(-1.0);
            } else if (gamepad1.dpad_down) {
                motor3.setPower(1.0);
            } else {
                motor3.setPower(0.0);
            }

            telemetry.addData("motor0/1 Power", motor0.getPower()); // motor0 and motor1 will have the same power
            telemetry.addData("motor2 Power", motor2.getPower());
            telemetry.addData("motor3 Power", motor3.getPower());
            telemetry.update();
        }
    }
}
