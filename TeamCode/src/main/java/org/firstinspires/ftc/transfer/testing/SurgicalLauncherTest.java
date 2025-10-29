package org.firstinspires.ftc.transfer.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class SurgicalLauncherTest extends LinearOpMode {
    DcMotor wheelLeft;
    DcMotor wheelRight;
    DcMotor intake;
    CRServo transfer;
    Servo trigger;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor backLeft;


    public static double drivingMult = 0.75;
    public static double turningMult = -0.8;
    public static double wheelSpeed = -1;
    public static double intakeSpeed = -1;
    public static double transferSpeed = -1;
    public static double triggerFlatPos = 0.28;
    public static double triggerLaunchPos = 0.95;

    public void runOpMode(){
        wheelLeft = hardwareMap.get(DcMotor.class, "wheelLeft");
        wheelRight = hardwareMap.get(DcMotor.class, "wheelRight");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(CRServo.class, "transfer");
        trigger = hardwareMap.get(Servo.class, "trigger");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()) {


            double driving = -gamepad1.right_stick_y * drivingMult; // Forward/backward
            double turning = gamepad1.left_stick_x * turningMult; // Turning
            double strafing = gamepad1.right_trigger - gamepad1.left_trigger; // Strafing

            // Combine inputs for each motor
            double frontLeftPower = driving + turning + strafing;
            double frontRightPower = driving - turning - strafing;
            double backLeftPower = driving + turning - strafing;
            double backRightPower = driving - turning + strafing;

            // Normalize powers to keep them within -1.0 to 1.0
            double maxPower = Math.max(1.0, Math.abs(frontLeftPower));
            maxPower = Math.max(maxPower, Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backLeftPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));

            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;

            // Set motor powers
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            if(gamepad1.a){
                wheelRight.setPower(wheelSpeed);
                wheelLeft.setPower(wheelSpeed);
            }

            if (gamepad1.b){
                wheelRight.setPower(0);
                wheelLeft.setPower(0);
            }

            if(gamepad1.x)
                intake.setPower(intakeSpeed);

            if (gamepad1.y)
                intake.setPower(0);

            if (gamepad1.dpad_up)
                transfer.setPower(transferSpeed);

            if (gamepad1.dpad_down)
                transfer.setPower(0);

            if (gamepad1.dpad_left)
                trigger.setPosition(triggerLaunchPos);

            if (gamepad1.dpad_right)
                trigger.setPosition(triggerFlatPos);
        }

    }
}
