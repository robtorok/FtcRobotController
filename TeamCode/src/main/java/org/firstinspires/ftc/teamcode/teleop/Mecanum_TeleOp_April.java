
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import android.util.Size;

@TeleOp
public class Mecanum_TeleOp_April extends LinearOpMode {

    private DcMotor intake_motor;
    private DcMotor bl_motor;
    private DcMotor br_motor;
    private DcMotor fl_motor;
    private DcMotor fr_motor;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;


    @Override
    public void runOpMode() {
        boolean intake_enabled;
        double driveFactor;
        double forward;
        double side;
        double turn;
        double denominator;

        intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");
        bl_motor = hardwareMap.get(DcMotor.class, "bl_motor");
        br_motor = hardwareMap.get(DcMotor.class, "br_motor");
        fl_motor = hardwareMap.get(DcMotor.class, "fl_motor");
        fr_motor = hardwareMap.get(DcMotor.class, "fr_motor");

        // Motor directions
        intake_motor.setDirection(DcMotor.Direction.REVERSE);
        bl_motor.setDirection(DcMotor.Direction.REVERSE);
        br_motor.setDirection(DcMotor.Direction.FORWARD);
        fl_motor.setDirection(DcMotor.Direction.REVERSE);
        fr_motor.setDirection(DcMotor.Direction.FORWARD);

        intake_enabled = false;

        initAprilTag();

        telemetry.addLine("AprilTag vision initialized.");
        telemetry.addLine("Press START");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Put loop blocks here.
            if (gamepad1.aWasPressed()) {
                intake_enabled = !intake_enabled;
            }

            if (gamepad1.left_bumper) {
                driveFactor = 0.25;
            } else if (gamepad1.right_bumper) {
                driveFactor = 1;
            } else {
                driveFactor = 0.5;
            }

            forward = driveFactor * -gamepad1.left_stick_y;
            side = driveFactor * gamepad1.left_stick_x;
            turn = driveFactor * gamepad1.right_stick_x;
            denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(forward), Math.abs(side), Math.abs(turn))), 1));
            fl_motor.setPower((forward + side + turn) / denominator);
            bl_motor.setPower(((forward - side) + turn) / denominator);
            fr_motor.setPower(((forward - side) - turn) / denominator);
            br_motor.setPower(((forward + side) - turn) / denominator);


            int tagCount = 0;
            if (aprilTag != null) {
                List<AprilTagDetection> detections = aprilTag.getDetections();
                tagCount = detections.size();   // number of tags currently detected
            }

            telemetry.addData("AprilTags Detected", tagCount);
            telemetry.addData("forward", forward);
            telemetry.addData("side", side);
            telemetry.addData("turn", turn);

            telemetry.update();
            if (intake_enabled) {
                intake_motor.setPower(1);
            } else {
                intake_motor.setPower(0);
            }
        }

    }

    private void initAprilTag() {
        // Create AprilTag processor (Builder pattern)
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create VisionPortal and attach AprilTag processor
        // Make sure your camera name matches the RC configuration name.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }
}