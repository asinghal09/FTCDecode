package org.firstinspires.ftc.teamcode.MeetCode.LM2;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
@Disabled
@TeleOp(name = "AprilTag Auto Heading Align")
public class AprilTagPipeline extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    private IMU imu;

    // ************ ADJUST THIS ************
    private static final double GOAL_OFFSET_DEG = 25; // angle from Tag to scoring opening

    @Override
    public void runOpMode() throws InterruptedException {

        // ------------------- IMU ------------------------
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                )
        );

        // ------------------- APRILTAG -------------------
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                //.setCamera(hardwareMap.get("camera"))
                .addProcessor(tagProcessor)
                .build();

        telemetry.addLine("Initialized. Press Start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            AprilTagDetection tag = getBestTag();

            if (tag != null) {
                // tag.ftcPose.yaw is the angle between ROBOT HEADING and TAG FACING DIRECTION.
                double tagYawDeg = Math.toDegrees(tag.ftcPose.yaw);

                // Add offset to aim at goal OPENING rather than tag
                double desiredHeadingDeg = tagYawDeg + GOAL_OFFSET_DEG;

                // Robot current heading from IMU (field-independent)
                double currentHeadingDeg = imu.getRobotYawPitchRollAngles().getYaw(
                        org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES
                );

                // Normalize and compute error
                double error = normalizeAngle(desiredHeadingDeg - currentHeadingDeg);

                double kP = 0.03; // adjust after testing
                double turnPower = Range.clip(error * kP, -0.4, 0.4);

                // Apply turning only
                double leftPower = turnPower;
                double rightPower = -turnPower;

                // Send power to your drivetrain (replace with your drive class)
                // For a 4-motor tank:
                //hardwareMap.get(DcMotor.class, "lf").setPower(leftPower);
                //hardwareMap.get(DcMotor.class, "lb").setPower(leftPower);
                //hardwareMap.get(DcMotor.class, "rf").setPower(rightPower);
                //hardwareMap.get(DcMotor.class, "rb").setPower(rightPower);

                telemetry.addLine("TAG FOUND");
                telemetry.addData("Tag Yaw", tagYawDeg);
                telemetry.addData("Desired Heading", desiredHeadingDeg);
                telemetry.addData("Current Heading", currentHeadingDeg);
                telemetry.addData("Error", error);
                telemetry.addData("Turn Power", turnPower);

            } else {
                telemetry.addLine("NO TAG VISIBLE — robot doesn't turn.");
            }

            telemetry.update();
        }
    }

    // Returns the best detected tag (largest or closest)
    private AprilTagDetection getBestTag() {
        if (tagProcessor.getDetections().isEmpty()) return null;
        return tagProcessor.getDetections().get(0);
    }

    // Normalize angle to [-180, 180]
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}
