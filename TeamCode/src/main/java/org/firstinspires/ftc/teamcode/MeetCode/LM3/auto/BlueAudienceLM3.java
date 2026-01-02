package org.firstinspires.ftc.teamcode.MeetCode.LM3.auto;

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

import org.firstinspires.ftc.teamcode.MeetCode.subsystems.Flywheels;
import org.firstinspires.ftc.teamcode.MeetCode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.MeetCode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.MeetCode.subsystems.Trigger;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;


@Config
@Autonomous
public class BlueAudienceLM3 extends LinearOpMode{
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

        Pose2d initialPose = mirror(new Pose2d(55, 16, Math.toRadians(-4)));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Flywheels flywheels = new Flywheels(hardwareMap);
        Trigger trigger = new Trigger(hardwareMap);


        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        double targetVoltage = 12.3;
        double correction = targetVoltage / batteryVoltageSensor.getVoltage();


        Action toGoal = drive.actionBuilder(initialPose)
                .setReversed(true)
                .splineToLinearHeading(mirror(new Pose2d(-28, 25,Math.toRadians(-52))), mirror(Math.toRadians(180)))
                .build();

        //1st line paths
        Action to1stLine = drive.actionBuilder(mirror(new Pose2d (-26, 23, Math.toRadians(-52))))
                .splineToSplineHeading(mirror(new Pose2d(-16,28, Math.toRadians(85))),mirror(Math.PI/2))
                .build();

        Action pickup1stLine = drive.actionBuilder(mirror(new Pose2d(-16,28, Math.toRadians(85))))
                .lineToY(mirror(48), new TranslationalVelConstraint(10))  // forward into 1st line of balls
                .build();

        Action toGoalWFirstLine = drive.actionBuilder(mirror(new Pose2d(-16, 48, Math.toRadians(85))))
                .setReversed(true)
                .splineToLinearHeading(mirror(new Pose2d(-28,25,Math.toRadians(-52))), mirror(26*Math.PI/35))
                .build();

        //2nd line path
        Action to2ndLine = drive.actionBuilder(mirror(new Pose2d (-28, 25, Math.toRadians(-52))))
                .setReversed(false)
                .splineToSplineHeading(mirror(new Pose2d(6.5,28, Math.toRadians(85))),mirror(Math.PI/2))
                .build();

        Action pickup2ndLine = drive.actionBuilder(mirror(new Pose2d(6.5,28, Math.toRadians(85))))
                .lineToY(mirror(48), new TranslationalVelConstraint(10)) // forward into 2nd line of balls
                .build();


        Action toGoalWSecondLine = drive.actionBuilder(mirror(new Pose2d(6.5, 48, Math.toRadians(85))))
                .setReversed(true)
                .splineToLinearHeading(mirror(new Pose2d(-28,25,Math.toRadians(-52))),mirror(26*Math.PI/35))
                .build();


        //3rd line path
        Action to3rdLine = drive.actionBuilder(mirror(new Pose2d (-28, 25, Math.toRadians(-52))))
                .setReversed(false)
                .splineToSplineHeading(mirror(new Pose2d(30,30, Math.toRadians(85))),mirror(0))
                .build();

        Action pickup3rdLine = drive.actionBuilder(mirror(new Pose2d(30,30, Math.toRadians(85))))
                .lineToY(mirror(48), new TranslationalVelConstraint(10)) // forward into 2nd line of balls
                .build();

        Action toGoalWThirdLine = drive.actionBuilder(mirror(new Pose2d(29, 48, Math.toRadians(85))))
                .setReversed(true)
                .splineToLinearHeading(mirror(new Pose2d(-28,25,Math.toRadians(-52))), mirror(26*Math.PI/35))
                .build();


        Action moveFromLine = drive.actionBuilder(mirror(new Pose2d(-30, 27, Math.toRadians(-52))))
                .splineToConstantHeading(mirror(new Vector2d(-16,45)), mirror(Math.toRadians(45)))
                .build();


        if (isStopRequested()) return;
        waitForStart();


        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(                 //launch preload
                                toGoal,
                                new SequentialAction(
                                        new SleepAction(0.15),
                                        flywheels.wheelsOn(correction)
                                )
                        ),
                        transfer.transferOn(),
                        intake.intakeOn(),
                        new SleepAction(0.5),
                        trigger.triggerUp(),
                        new SleepAction(0.5),
                        flywheels.wheelsOff(),
                        trigger.triggerDown(),
                        intake.intakeOff(),
                        transfer.transferOff(),

                        //1st line
                        new ParallelAction(
                                to1stLine,
                                intake.intakeOn(),              //intake 1st line
                                transfer.transferOnSlow()
                        ),
                        new ParallelAction(
                                pickup1stLine,           //intake 1st line
                                new SequentialAction(
                                        new SleepAction(2),
                                        transfer.transferOff()
                                )
                        ),
                        new SleepAction(0.25),
                        intake.intakeOff(),
                        new ParallelAction(                 //launch 1st line
                                toGoalWFirstLine,
                                new SequentialAction(
                                        new SleepAction(1),
                                        flywheels.wheelsOn(correction)
                                )

                        ),
                        transfer.transferOn(),
                        intake.intakeOn(),
                        new SleepAction(0.5),
                        trigger.triggerUp(),
                        new SleepAction(0.5),

                        flywheels.wheelsOff(),
                        trigger.triggerDown(),
                        intake.intakeOff(),
                        transfer.transferOff(),


                        //2nd line
                        new ParallelAction(
                                to2ndLine,
                                intake.intakeOn(),              //intake 2nd line
                                transfer.transferOnSlow()
                        ),
                        new ParallelAction(
                                pickup2ndLine,
                                new SequentialAction(
                                        new SleepAction(2),
                                        transfer.transferOff()
                                )
                        ),
                        new SleepAction(0.25),
                        intake.intakeOff(),
                        new ParallelAction(                      //launch 2nd line
                                toGoalWSecondLine,
                                new SequentialAction(
                                        new SleepAction(2),
                                        flywheels.wheelsOn(correction)
                                )
                        ),
                        transfer.transferOn(),
                        intake.intakeOn(),
                        new SleepAction(0.5),
                        trigger.triggerUp(),
                        new SleepAction(0.5),

                        flywheels.wheelsOff(),
                        trigger.triggerDown(),
                        intake.intakeOff(),
                        transfer.transferOff(),


                        //3rd line
                        new ParallelAction(
                                to3rdLine,
                                intake.intakeOn(),              //intake 3rd line
                                transfer.transferOnSlow()
                        ),
                        new ParallelAction(
                                pickup3rdLine,
                                new SequentialAction(
                                        new SleepAction(1.5),
                                        transfer.transferOff()
                                )
                        ),
                        new SleepAction(0.25),
                        intake.intakeOff(),
                        new ParallelAction(                      //launch 3rd line
                                toGoalWThirdLine,
                                new SequentialAction(
                                        new SleepAction(2),
                                        flywheels.wheelsOn(correction)
                                )
                        ),
                        transfer.transferOn(),
                        intake.intakeOn(),
                        new SleepAction(0.5),
                        trigger.triggerUp(),
                        new SleepAction(0.5),

                        flywheels.wheelsOff(),
                        trigger.triggerDown(),
                        intake.intakeOff(),
                        transfer.transferOff(),
                        moveFromLine
                )
        );

    }
}
