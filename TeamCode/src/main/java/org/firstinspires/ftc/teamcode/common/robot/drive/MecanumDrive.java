package org.firstinspires.ftc.teamcode.common.robot.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrive {

    private DcMotor driveFL, driveFR, driveBL, driveBR;

    public MecanumDrive(HardwareMap hardwareMap) {
        driveFL = hardwareMap.get(DcMotor.class, "driveFL");
        driveFR = hardwareMap.get(DcMotor.class, "driveFR");
        driveBL = hardwareMap.get(DcMotor.class, "driveBL");
        driveBR = hardwareMap.get(DcMotor.class, "driveBR");

        driveFL.setDirection(DcMotor.Direction.REVERSE);
        driveFR.setDirection(DcMotor.Direction.FORWARD);
        driveBL.setDirection(DcMotor.Direction.REVERSE);
        driveBR.setDirection(DcMotor.Direction.FORWARD);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        driveFL.setZeroPowerBehavior(behavior);
        driveFR.setZeroPowerBehavior(behavior);
        driveBL.setZeroPowerBehavior(behavior);
        driveBR.setZeroPowerBehavior(behavior);
    }

    public void setMotorPowers(double fl, double fr, double bl, double br) {
        driveFL.setPower(fl);
        driveFR.setPower(fr);
        driveBL.setPower(bl);
        driveBR.setPower(br);
    }

    public void drive(double axial, double lateral, double yaw) {
        double powerFL = axial + lateral + yaw;
        double powerFR = axial - lateral - yaw;
        double powerBL = axial - lateral + yaw;
        double powerBR = axial + lateral - yaw;

        double max = Math.max(Math.abs(powerFL), Math.abs(powerFR));
        max = Math.max(max, Math.abs(powerBL));
        max = Math.max(max, Math.abs(powerBR));

        if (max > 1.0) {
            powerFL /= max;
            powerFR /= max;
            powerBL /= max;
            powerBR /= max;
        }

        setMotorPowers(powerFL, powerFR, powerBL, powerBR);
    }
}
