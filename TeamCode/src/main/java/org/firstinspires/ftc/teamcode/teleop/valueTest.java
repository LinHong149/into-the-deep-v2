package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp
public class valueTest extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    DcMotorEx S1Motor, S2Motor, AMotor, FL, FR, BR, BL;
    Servo wrist,rotation,claw;
    static double wristVal,rotationVal,clawVal;
    List<Double> values = new ArrayList<>();
    public void initHardware() {
//      DRIVE MOTORS
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        AMotor = hardwareMap.get(DcMotorEx.class,"AMotor");
        S1Motor = hardwareMap.get(DcMotorEx.class,"S1Motor");
        S2Motor = hardwareMap.get(DcMotorEx.class,"S2Motor");

        rotation = hardwareMap.get(Servo.class,"rotation");
        wrist = hardwareMap.get(Servo.class,"wrist");
        claw = hardwareMap.get(Servo.class,"claw");

        FL.setDirection(DcMotorEx.Direction.FORWARD);
        BL.setDirection(DcMotorEx.Direction.FORWARD);
        FR.setDirection(DcMotorEx.Direction.REVERSE);
        BR.setDirection(DcMotorEx.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);


        AMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        AMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        AMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        AMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        AMotor.setPower(0);

        S1Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        S1Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        S1Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        S1Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        S1Motor.setPower(0);

        S2Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        S2Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        S2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        S2Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        S2Motor.setPower(0);
    }

    @Override
    public void runOpMode(){
        initHardware();

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            FL.setPower(frontLeftPower);
            BL.setPower(backLeftPower);
            FR.setPower(frontRightPower);
            BR.setPower(backRightPower);

            wrist.setPosition(wristVal);
            rotation.setPosition(rotationVal);
            claw.setPosition(clawVal);
            values.clear();
            values.add(wristVal);
            values.add(clawVal);
            values.add(rotationVal);
            dashboardTelemetry.addData("values",values);


        }
    }


}
