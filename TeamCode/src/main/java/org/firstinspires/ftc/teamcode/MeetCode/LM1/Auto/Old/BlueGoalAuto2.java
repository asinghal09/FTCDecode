package org.firstinspires.ftc.teamcode.MeetCode.LM1.Auto.Old;

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MeetCode.LM1.Auto.subsystems.Flywheels;
import org.firstinspires.ftc.teamcode.MeetCode.LM1.Auto.subsystems.Intake;
import org.firstinspires.ftc.teamcode.MeetCode.LM1.Auto.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.MeetCode.LM1.Auto.subsystems.Trigger;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
@Disabled

@Config
@Autonomous
public class BlueGoalAuto2 extends LinearOpMode{

    private static final Logger log = LoggerFactory.getLogger(BlueGoalAuto2.class);


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
    @Override
    public void runOpMode(){

        Pose2d initialPose = mirror(new Pose2d(-55, 52, Math.toRadians(-57)));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Flywheels flywheels = new Flywheels(hardwareMap);
        Trigger trigger = new Trigger(hardwareMap);


        Action backFromGoal = drive.actionBuilder(initialPose)
                .splineToLinearHeading(mirror(new Pose2d(-30, 27,Math.toRadians(-52))), mirror(Math.toRadians(-52)))
                .build();

        Action to1stLine = drive.actionBuilder(mirror(new Pose2d (-26, 23, Math.toRadians(-52))))
                .splineToSplineHeading(mirror(new Pose2d(-16,28, Math.toRadians(85))),mirror(Math.PI/2), new TranslationalVelConstraint(20) )
                .build();

        Action pickup1stLine = drive.actionBuilder(mirror(new Pose2d(-16,28, Math.toRadians(85))))
                .lineToY(mirror(45), new TranslationalVelConstraint(4)) // forward into 1st line of balls
                .build();

        Action toGoalWFirstLine = drive.actionBuilder(mirror(new Pose2d(-16, 45, Math.toRadians(85))))
                .strafeTo(mirror(new Vector2d(-17, 23)))
                .splineToLinearHeading(mirror(new Pose2d(-30,27,Math.toRadians(-52))), mirror(26*Math.PI/35))
                .build();

        Action to2ndLine = drive.actionBuilder(mirror(new Pose2d (-30, 27, Math.toRadians(-52))))
                .splineToSplineHeading(mirror(new Pose2d(7,28, Math.toRadians(85))),mirror(Math.PI/2), new TranslationalVelConstraint(20) )
                .build();

        Action pickup2ndLine = drive.actionBuilder(mirror(new Pose2d(7,28, Math.toRadians(85))))
                .lineToY(mirror(45), new TranslationalVelConstraint(4)) // forward into 1st line of balls
                .build();


        Action toGoalWSecondLine = drive.actionBuilder(mirror(new Pose2d(7, 45, Math.toRadians(85))))
                .strafeTo(mirror(new Vector2d(0, 23)))
                .splineToLinearHeading(mirror(new Pose2d(-30,27,Math.toRadians(-52))), mirror(26*Math.PI/35))
                .build();



        if (isStopRequested()) return;
        trigger.triggerDown();
        waitForStart();


        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                backFromGoal,
                                flywheels.wheelsOn()
                        ),
                        transfer.transferOn(),          //launch preload
                        intake.intakeOn(),
                        new SleepAction(4),
                        trigger.triggerUp(),
                        new SleepAction(1),
                        flywheels.wheelsOff(),
                        trigger.triggerDown(),
                        to1stLine,
                        intake.intakeOn(),              //intake 1st line
                        transfer.transferOn(),
                        pickup1stLine,
                        new SleepAction(0.25),
                        transfer.transferOff(),
                        new SleepAction(0.75),
                        //intake.intakeOff(),
                        new ParallelAction(
                                toGoalWFirstLine,
                                flywheels.wheelsOn()
                        ),
                        transfer.transferOn(),          //launch 1st line
                        //intake.intakeOn(),
                        new SleepAction(4),
                        trigger.triggerUp(),
                        new SleepAction(1),
                        flywheels.wheelsOff(),
                        trigger.triggerDown(),
                        to2ndLine,
                        intake.intakeOn(),              //intake 2nd line
                        transfer.transferOn(),
                        pickup2ndLine,
                        new SleepAction(0.25),
                        transfer.transferOff(),
                        new SleepAction(0.75)

                )
        );


    }
}
