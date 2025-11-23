package org.firstinspires.ftc.teamcode.MeetCode.LM2.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MeetCode.LM1.Auto.subsystems.Flywheels;
import org.firstinspires.ftc.teamcode.MeetCode.LM1.Auto.subsystems.Intake;
import org.firstinspires.ftc.teamcode.MeetCode.LM1.Auto.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.MeetCode.LM1.Auto.subsystems.Trigger;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;


@Config
@Autonomous
public class BlueGoal extends LinearOpMode{

    private Pose2d mirror(Pose2d p) {
        return new Pose2d(p.position.x,-p.position.y, -p.heading.toDouble());
    }

    private Vector2d mirror(Vector2d v) {
        return new Vector2d(
                v.x,      // X same
                -v.y      // Y flipped
        );
    }
    private double mirror(double y){
        return -y;
    }
    private VoltageSensor batteryVoltageSensor;


    @Override
    public void runOpMode(){

        Pose2d initialPose = mirror(new Pose2d(-55, 52, Math.toRadians(-52)));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Flywheels flywheels = new Flywheels(hardwareMap);
        Trigger trigger = new Trigger(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        double targetVoltage = 12.3;
        double correction = targetVoltage / batteryVoltageSensor.getVoltage();


        Action backFromGoal = drive.actionBuilder(initialPose)
                .splineToLinearHeading(mirror(new Pose2d(-28, 25,Math.toRadians(-52))), mirror(Math.toRadians(-52)))
                .build();

        Action to1stLine = drive.actionBuilder(mirror(new Pose2d (-26, 23, Math.toRadians(-52))))
                .splineToSplineHeading(mirror(new Pose2d(-16,28, Math.toRadians(85))),mirror(Math.PI/2))
                .build();

        Action pickup1stLine = drive.actionBuilder(mirror(new Pose2d(-16,28, Math.toRadians(85))))
                .lineToY(mirror(38), new TranslationalVelConstraint(6)) // forward into 1st line of balls

                .lineToY(mirror(48), new TranslationalVelConstraint(4)) // forward into 1st line of balls
                .build();

        Action toGoalWFirstLine = drive.actionBuilder(mirror(new Pose2d(-16, 48, Math.toRadians(85))))
                .setReversed(true)
                //.splineToLinearHeading(new Pose2d(-20, 23, Math.toRadians(30)), Math.toRadians(210))
                .splineToLinearHeading(mirror(new Pose2d(-28,25,Math.toRadians(-52))), mirror(26*Math.PI/35))
                .build();

        Action to2ndLine = drive.actionBuilder(mirror(new Pose2d (-30, 27, Math.toRadians(-52))))
                .setReversed(false)
                .splineToSplineHeading(mirror(new Pose2d(6.5,28, Math.toRadians(85))), mirror(Math.PI/2))
                .build();

        Action pickup2ndLine = drive.actionBuilder(mirror(new Pose2d(6.5,28, Math.toRadians(85))))
                .lineToY(mirror(38), new TranslationalVelConstraint(6)) // forward into 1st line of balls
                .lineToY(mirror(48), new TranslationalVelConstraint(4)) // forward into 1st line of balls

                .build();


        Action toGoalWSecondLine = drive.actionBuilder(mirror(new Pose2d(6.5, 48, Math.toRadians(85))))
                .setReversed(true)
                //.splineTo(new Vector2d(0, 23), Math.toRadians(-90))
                .splineToLinearHeading(mirror(new Pose2d(-28,25, Math.toRadians(-52))), mirror(26*Math.PI/35))
                .build();
        Action moveFromLine = drive.actionBuilder(mirror(new Pose2d(-30, 27, Math.toRadians(-52))))
                .splineToConstantHeading(mirror(new Vector2d(-16,45)), mirror(Math.toRadians(45)))
                .build();


        if (isStopRequested()) return;
        trigger.triggerDown();
        waitForStart();


        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(                 //launch preload
                                backFromGoal,
                                flywheels.wheelsOn(correction),
                                new SleepAction(0.15),
                                transfer.transferOn(),
                                intake.intakeOn()
                        ),
                        new SleepAction(1.5),
                        trigger.triggerUp(),
                        new SleepAction(1),
                        flywheels.wheelsOff(),
                        trigger.triggerDown(),
                        new ParallelAction(
                            to1stLine,
                            intake.intakeOn(),              //intake 1st line
                            transfer.transferOn()
                        ),

                        pickup1stLine,
                        new SleepAction(0.15),
                        transfer.transferOff(),
                        new SleepAction(0.25),
                        //intake.intakeOff(),
                        new ParallelAction(                 //launch 1st line
                                toGoalWFirstLine,
                                flywheels.wheelsOn(correction),
                                new SleepAction(1.9),
                                transfer.transferOn()
                        ),
                        new SleepAction(1.75),
                        trigger.triggerUp(),
                        new SleepAction(1),
                        flywheels.wheelsOff(),
                        trigger.triggerDown(),
                        new ParallelAction(
                                to2ndLine,
                                intake.intakeOn(),              //intake 2nd line
                                transfer.transferOn()
                        ),
                        pickup2ndLine,
                        new SleepAction(0.1),
                        transfer.transferOff(),
                        new SleepAction(0.25),
                        new ParallelAction(                 //launch 2nd line
                                toGoalWSecondLine,
                                flywheels.wheelsOn(correction),
                                new SleepAction(5.75),
                                transfer.transferOn()
                        ),
                        trigger.triggerUp(),
                        new SleepAction(0.75),
                        moveFromLine

                        /*trigger.triggerUp(),
                        new SleepAction(0.5),

                        flywheels.wheelsOff(),
                        trigger.triggerDown()

                         */

                )
        );

    }
}
