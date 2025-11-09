package org.firstinspires.ftc.teamcode.MeetCode.LM1;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
@Config
public class oneControllerTeleop extends LinearOpMode {
    DcMotor wheelLeft;
    DcMotor wheelRight;
    DcMotor intake;
    DcMotor transfer;
    Servo trigger;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor backLeft;

    DistanceSensor distanceSensor;


    public static double drivingMult = 0.75;
    public static double turningMult = 0.8;
    public static double wheelSpeed = -0.95;
    public static double intakeSpeed = -1;
    public static double transferSpeed = 1;
    public static double triggerFlatPos = 0.42;
    public static double triggerLaunchPos = 0.94;

    public static double targetDistance = 120;

    boolean isIntaking = false;
    boolean previousXState = false;
    boolean isLaunching = false;
    boolean previousAState = false;

    boolean launching = false;
    public void runOpMode(){
        wheelLeft = hardwareMap.get(DcMotor.class, "wheelLeft");
        wheelRight = hardwareMap.get(DcMotor.class, "wheelRight");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        trigger = hardwareMap.get(Servo.class, "trigger");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");


        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelRight.setDirection(DcMotorSimple.Direction.REVERSE);

        wheelRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        wheelLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        trigger.setPosition(triggerFlatPos);

        waitForStart();

        while(opModeIsActive()) {

            double driving = -gamepad1.right_stick_y * drivingMult; // Forward/backward
            double turning = gamepad1.right_stick_x * turningMult; // Turning
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


            //intake toggle
            boolean currentXState = gamepad1.x;

            if (currentXState && !previousXState){
                isIntaking = !isIntaking;
                if (isIntaking)
                    intake.setPower(intakeSpeed);
                else
                    intake.setPower(0);
            }
            previousXState = currentXState;

            //wheel toggle
            boolean currentAState = gamepad1.a;

            if (currentAState && !previousAState){
                isLaunching = !isLaunching;
                if(isLaunching){
                    wheelRight.setPower(wheelSpeed);
                    wheelLeft.setPower(wheelSpeed);
                }

                else {
                    wheelRight.setPower(0);
                    wheelLeft.setPower(0);
                }
            }
            previousAState = currentAState;

            if(gamepad1.y){
                intake.setPower(intakeSpeed);
                transfer.setPower(transferSpeed);
                wheelRight.setPower(0);
                wheelLeft.setPower(0);
                isLaunching = false;
                isIntaking = true;
            }
            if (gamepad1.b){
                wheelRight.setPower(wheelSpeed);
                wheelLeft.setPower(wheelSpeed);
                intake.setPower(intakeSpeed);
                transfer.setPower(transferSpeed);
                isIntaking = true;
                isLaunching = true;
            }

            if (gamepad1.dpad_up)
                transfer.setPower(transferSpeed);

            if (gamepad1.dpad_down)
                transfer.setPower(0);
            if(gamepad1.dpad_left){
                transfer.setPower(-transferSpeed);
            }

            if (gamepad1.right_bumper)
                trigger.setPosition(triggerLaunchPos);

            if (gamepad1.left_bumper)
                trigger.setPosition(triggerFlatPos);

            if (gamepad1.back){
                transfer.setPower(0);
                wheelLeft.setPower(0);
                wheelRight.setPower(0);
                intake.setPower(0);
                trigger.setPosition(triggerFlatPos);
                isIntaking = false;
                isLaunching = false;

            }

            //Distance sensor stuff
            double distance = distanceSensor.getDistance(DistanceUnit.CM);


            if(gamepad1.guide){
                launching = true;
            }

            if (launching){
                if(distance > targetDistance) {
                    while (distance > targetDistance) {
                        frontLeftPower = -0.4;
                        backLeftPower = -0.4;
                        frontRightPower = -0.4;
                        backRightPower = -0.4;

                        // Set motor powers
                        frontLeft.setPower(frontLeftPower);
                        frontRight.setPower(frontRightPower);
                        backLeft.setPower(backLeftPower);
                        backRight.setPower(backRightPower);

                        distance = distanceSensor.getDistance(DistanceUnit.CM);

                    }
                    frontLeft.setPower(0);
                    backLeft.setPower(0);
                    frontRight.setPower(0);
                    backRight.setPower(0);
                    launching = false;
                }
                else if (distance < targetDistance - 35){
                    while (distance < targetDistance -35) {
                        frontLeftPower = 0.4;
                        backLeftPower = 0.4;
                        frontRightPower = 0.4;
                        backRightPower = 0.4;

                        // Set motor powers
                        frontLeft.setPower(frontLeftPower);
                        frontRight.setPower(frontRightPower);
                        backLeft.setPower(backLeftPower);
                        backRight.setPower(backRightPower);

                        distance = distanceSensor.getDistance(DistanceUnit.CM);

                    }
                    frontLeft.setPower(0);
                    backLeft.setPower(0);
                    frontRight.setPower(0);
                    backRight.setPower(0);
                    launching = false;
                }

            }

            telemetry.addData("distance", distance);
            telemetry.update();

        }

    }
}
