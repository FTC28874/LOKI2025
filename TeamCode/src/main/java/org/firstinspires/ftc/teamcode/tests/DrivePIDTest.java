package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.robot.drive.MecanumDrive;

@TeleOp(name="Drive PID Test", group="Test")
public class DrivePIDTest extends LinearOpMode {

    private MecanumDrive drive;

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap);

        telemetry.addLine("Drive Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x) {
                drive.setTargetPose(new Pose2D(DistanceUnit.MM, 1000, 0, AngleUnit.DEGREES, 0)); // Move 1000 mm forward
            } else if (gamepad1.a) {
                drive.setManualMode();
            }

            if (drive.getMode() == MecanumDrive.DriveMode.MANUAL) {
                double axial = -gamepad1.left_stick_y;
                double lateral = gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x;
                drive.driveFieldCentric(axial, lateral, yaw);
            } else {
                drive.updateAuto();
            }

            Pose2D pos = drive.getPose();
            telemetry.addData("Mode", drive.getMode());
            telemetry.addData("X", pos.getX(DistanceUnit.MM));
            telemetry.addData("Y", pos.getY(DistanceUnit.MM));
            telemetry.addData("Heading", pos.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}
