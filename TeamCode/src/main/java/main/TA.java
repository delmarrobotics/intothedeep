/*
 * This file contains a OpMode for the autonomous phase when the robot starts at
 * the blue observation position.
 *
 */

package main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import common.Robot;

@Autonomous(name="T A", group="Main")
public class TA extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the robot hardware.
        Robot robot = new Robot(this);
        robot.init();
        //robot.startVision();

        //Auto auto = new Auto(this, robot);

        /*telemetry.addLine("waiting for camera");
        telemetry.update();
        auto.waitForCamera();
        telemetry.addLine("camera ready, press start"); //sleep(200) not needed here
        telemetry.update();*/

        //robot.vision.enableCameraStream(true);

        waitForStart();
        runtime.reset();
        //elbow -3224

        //robot.vision.enableCameraStream(false);
        //paused for new arm

        DcMotor.RunMode mode = robot.drive.leftFrontDrive.getMode();
        for (DcMotor motor : robot.drive.motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        robot.drive.leftFrontDrive.setTargetPosition(100);
        robot.drive.rightFrontDrive.setTargetPosition(-100);
        robot.drive.leftBackDrive.setTargetPosition(100);
        robot.drive.rightBackDrive.setTargetPosition(-100);

        // Turn On RUN_TO_POSITION
        for (DcMotor motor : robot.drive.motors)
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.drive.leftFrontDrive.setPower(1);
        robot.drive.rightFrontDrive.setPower(1);
        robot.drive.leftBackDrive.setPower(1);
        robot.drive.rightBackDrive.setPower(1);

        sleep(30000);
        /*robot.forward(24);
        sleep(1000);
        robot.back(24);
        sleep(1000);*/
    }
}
