package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@TeleOp
public class OtherRobotTesting extends LinearOpMode{
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor backLeft;

    //DcMotor LWheel;
    //DcMotor RWheel;
    Servo intakeRight;
    Servo intakeLeft;


    public static double drivingMult = 0.75;
    public static double turningMult = -0.8;
    public static double wheelSpeed = 1;
    public static double intake1pos = 0.5;
    public static double intake2pos = 0.5;

    public void runOpMode(){
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        //LWheel = hardwareMap.get(DcMotor.class, "LWheel");
        //RWheel = hardwareMap.get(DcMotor.class, "RWheel");
        //RWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeRight = hardwareMap.get(Servo.class,   "intakeRight");
        intakeLeft = hardwareMap.get(Servo.class,   "intakeLeft");



        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        while(opModeIsActive()){

            double driving = -gamepad1.right_stick_y * drivingMult; // Forward/backward
            double turning = gamepad1.left_stick_x * turningMult; // Turning
            double strafing = gamepad1.right_trigger - gamepad1.left_trigger; // Strafing (disabled rn)

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
                //LWheel.setPower(wheelSpeed);
                //RWheel.setPower(wheelSpeed);
            }

            if (gamepad1.b){
                //LWheel.setPower(0);
                //RWheel.setPower(0);
            }

            if(gamepad1.x){
                intakeRight.setPosition(intake1pos);
                intakeLeft.setPosition(intake2pos);
            }




            telemetry.addData("Front Left Power", frontLeft.getPower());
            telemetry.addData("Front Right Power", frontRight.getPower());
            telemetry.addData("Back Left Power", backLeft.getPower());
            telemetry.addData("Back Right Power", backRight.getPower());
            telemetry.update();




        }

    }


}
