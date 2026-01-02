package org.firstinspires.ftc.teamcode.MeetCode.LM3;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;
@TeleOp
@Config
public class TeleOp1_4ManualSpeed extends LinearOpMode {
    DcMotor wheelLeft;
    DcMotor wheelRight;
    DcMotor intake;
    DcMotor transfer;
    Servo trigger;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor backLeft;


    private VoltageSensor batteryVoltageSensor;


    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    public static double drivingMult = 0.75;
    public static double turningMult = 0.8;
    public static double wheelSpeed = 0.95;
    public static double intakeSpeed = -1;
    public static double transferSpeed = 0.35;
    public static double triggerFlatPos = 0.42;
    public static double triggerLaunchPos = 0.94;
    public static double flywheelRatioMult = 0.839951541;

    double distance;

    public static double targetDistance = 120;

    public static double targetVoltage = 12.3;

    boolean isIntaking = false;
    boolean previousXState = false;
    boolean isLaunching = false;
    boolean previousAState = false;

    public static double turnCorrectionSpeed = 0.3;
    public static long turnTime = 50;

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


        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        wheelRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        wheelLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        trigger.setPosition(triggerFlatPos);


        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();


        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(visionPortal, 30);
        telemetry.addLine("AprilTag Vision Initialized");
        telemetry.update();


        waitForStart();

        while(opModeIsActive()) {
            double voltage = batteryVoltageSensor.getVoltage();
            double correction = targetVoltage/voltage;
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


            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (detections.size() == 0) {
                telemetry.addLine("No AprilTag detected");
            } else {
                for (AprilTagDetection tag : detections) {

                    telemetry.addLine("Tag Detected!");
                    telemetry.addData("Tag ID", tag.id);

                    // Distance straight ahead to tag (goal distance)
                    telemetry.addData("Distance (in)",
                            "%.2f", tag.ftcPose.z);

                    // Horizontal offset (left/right)
                    telemetry.addData("X Offset (in)",
                            "%.2f", tag.ftcPose.x);

                    // Rotation relative to tag
                    telemetry.addData("Yaw (deg)",
                            "%.2f", tag.ftcPose.yaw);
                }
            }


            //intake toggle
            boolean currentXState = gamepad2.x;

            if (currentXState && !previousXState){
                isIntaking = !isIntaking;
                if (isIntaking)
                    intake.setPower(intakeSpeed);
                else
                    intake.setPower(0);
            }
            previousXState = currentXState;

            //wheel toggle
            boolean currentAState = gamepad2.a;

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

            if(gamepad2.y){
                intake.setPower(intakeSpeed);
                transfer.setPower(transferSpeed);
                wheelRight.setPower(0);
                wheelLeft.setPower(0);
                isLaunching = false;
                isIntaking = true;
            }
            if (gamepad2.b){

                intake.setPower(intakeSpeed);
                transfer.setPower(1);
                isIntaking = true;
                isLaunching = true;
            }


            if(isLaunching){


                /*
                wheelSpeed = (0.00344595 * distance + 0.544257) * correction;
                wheelSpeed *= flywheelRatioMult;


                if(wheelSpeed > (0.92 * correction)){
                    wheelSpeed = 0.92 * correction;
                }

                 */


                wheelLeft.setPower(wheelSpeed);
                wheelRight.setPower(wheelSpeed);
            }


            if (gamepad2.dpad_up)
                transfer.setPower(transferSpeed);

            if (gamepad2.dpad_down)
                transfer.setPower(0);
            if(gamepad2.dpad_left){
                transfer.setPower(-transferSpeed);
            }

            if (gamepad2.right_bumper)
                trigger.setPosition(triggerLaunchPos);

            if (gamepad2.left_bumper)
                trigger.setPosition(triggerFlatPos);

            if (gamepad2.back){
                transfer.setPower(0);
                wheelLeft.setPower(0);
                wheelRight.setPower(0);
                intake.setPower(0);
                trigger.setPosition(triggerFlatPos);
                isIntaking = false;
                isLaunching = false;

            }


            if(gamepad2.guide){
                wheelLeft.setPower(-0.92 * correction);
                wheelRight.setPower(-0.92 * correction);
            }



            if(gamepad2.right_trigger > 0.5) {
                intake.setPower(-intakeSpeed);
            }

            if (gamepad1.dpad_right) {
                frontLeft.setPower(turnCorrectionSpeed);
                backLeft.setPower(turnCorrectionSpeed);
                frontRight.setPower(-turnCorrectionSpeed);
                backRight.setPower(-turnCorrectionSpeed);

                sleep(turnTime);

                frontLeft.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);


            }

            if (gamepad1.dpad_left) {
                frontLeft.setPower(-turnCorrectionSpeed);
                backLeft.setPower(-turnCorrectionSpeed);
                frontRight.setPower(turnCorrectionSpeed);
                backRight.setPower(turnCorrectionSpeed);

                sleep(turnTime);

                frontLeft.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);


            }

            if (gamepad1.dpad_up) {
                frontLeft.setPower(turnCorrectionSpeed);
                backLeft.setPower(turnCorrectionSpeed);
                frontRight.setPower(turnCorrectionSpeed);
                backRight.setPower(turnCorrectionSpeed);

                sleep(turnTime);

                frontLeft.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
            }
            if (gamepad1.dpad_down) {
                frontLeft.setPower(-turnCorrectionSpeed);
                backLeft.setPower(-turnCorrectionSpeed);
                frontRight.setPower(-turnCorrectionSpeed);
                backRight.setPower(-turnCorrectionSpeed);

                sleep(turnTime);

                frontLeft.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
            }


            telemetry.addData("flywheel speed", wheelLeft.getPower());
            telemetry.addData("voltage", voltage);
            telemetry.addData("correction", correction);
            telemetry.update();

        }
        visionPortal.close();

    }
}
