package org.firstinspires.ftc.teamcode.MeetCode.LM2.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
public class BlueAudience extends LinearOpMode{

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

        Pose2d initialPose = mirror(new Pose2d(53, 18, Math.toRadians(-4)));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Flywheels flywheels = new Flywheels(hardwareMap);
        Trigger trigger = new Trigger(hardwareMap);


        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        double targetVoltage = 12.3;
        double correction = targetVoltage / batteryVoltageSensor.getVoltage();


        Action backFromGoal = drive.actionBuilder(initialPose)
                .setReversed(true)
                .splineToLinearHeading(mirror(new Pose2d(-28, 25,Math.toRadians(-52))), mirror(Math.toRadians(128)))
                .build();

        Action back = drive.actionBuilder(mirror(new Pose2d (-26, 23, Math.toRadians(-52))))
                .setReversed(false)
                .splineToConstantHeading(mirror(new Vector2d(0,24)),mirror(0))
                .build();


        if (isStopRequested()) return;
        waitForStart();


        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(                 //launch preload
                                backFromGoal,
                                flywheels.wheelsOn(correction)
                        ),
                        new SleepAction(1),
                        transfer.transferOn(),
                        new SleepAction(0.2),
                        intake.intakeOn(),
                        new SleepAction(2.5),
                        trigger.triggerUp(),
                        new SleepAction(1),
                        flywheels.wheelsOff(),
                        trigger.triggerDown(),
                        back

                )
        );

    }
}
