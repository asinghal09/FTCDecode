package org.firstinspires.ftc.teamcode.MeetCode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
public class Flywheels {
    private DcMotorEx leftFlywheel, rightFlywheel;
    private VoltageSensor batteryVoltageSensor;
    public static double fWheelPower = 0.8;
    public static double targetVoltage = 12.3;

    public Flywheels(HardwareMap hardwareMap) {
        leftFlywheel  = hardwareMap.get(DcMotorEx.class, "wheelLeft");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "wheelRight");


        // Reverse one side so they spin in the same physical direction
        leftFlywheel.setDirection(DcMotorEx.Direction.REVERSE);


        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    /*
    public void setPower(double p) {
        power = p;
        leftFlywheel.setPower(p);
        rightFlywheel.setPower(p);
    }

     */

    public Action wheelsOn(double correction) {
        return t -> {
            leftFlywheel.setPower(fWheelPower * correction);
            rightFlywheel.setPower(fWheelPower * correction);
            return false;   // run once
        };
    }
    public Action wheelsOnSpeed(double speed) {
        return t -> {
            leftFlywheel.setPower(speed);
            rightFlywheel.setPower(speed);
            return false;   // run once
        };
    }
    public Action wheelsOff() {
        return t -> {
            leftFlywheel.setPower(0);
            rightFlywheel.setPower(0);
            return false;   // run once
        };
    }



}
