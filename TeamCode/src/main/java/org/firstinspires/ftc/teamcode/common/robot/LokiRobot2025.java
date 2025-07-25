package org.firstinspires.ftc.teamcode.common.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.robot.drive.MecanumDrive;

public class LokiRobot2025 {
    HardwareMap mainMap;

    public LiftPivot liftPivot;
    public Arm arm;
    public MecanumDrive drive;
    private ElapsedTime robotTimer;

    public LokiRobot2025(HardwareMap hMap, boolean resetEncoder){
        init(hMap, resetEncoder);
    }

    public void init(HardwareMap hMap, boolean resetEncoder){
        mainMap = hMap;
        liftPivot = new LiftPivot(hMap, resetEncoder);
        arm = new Arm(hMap);
        drive = new MecanumDrive(hMap);
        robotTimer = new ElapsedTime();
    }

    public void update(){
        liftPivot.update();
    }
}
