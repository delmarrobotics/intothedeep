/*
 * This file contains a OpMode for the autonomous phase when the robot starts at
 * the blue observation position.
 *
 */

package main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import common.Arm;
import common.Robot;

@Autonomous(name="Sample Bucket Claw Auto", group="Main")
public class SampleBucketClawAuto extends LinearOpMode {

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
        robot.strafeRight(25.5);
        robot.arm.elbowMove(-2376);
        sleep(500);
        robot.arm.armMove(robot.arm.ARM_OUT_HIGH);
        sleep(2500);
        robot.forward(26);
        robot.arm.setSpecimen(false);
        sleep(300);
        robot.back(3);
        robot.arm.armMove(0);
        sleep(1000);
        robot.arm.elbowMove(0);
        sleep(1500);
        robot.forward(3);
        robot.turn(-150);
        //robot.arm.wristMove(Arm.intakeStates.FORWARD);
        //sleep(500);
        robot.forward(10);
        robot.arm.armMove(-2400);
        robot.arm.elbowMove(0);
        sleep(750);
        robot.arm.setSpecimen(true);
        robot.arm.elbowMove(0);
        sleep(500);
        robot.arm.armMove(0);
        robot.back(10);
        sleep(1000);
        robot.turn(150);
        robot.back(7);
        //robot.arm.wristMove(Arm.intakeStates.REVERSE);
        robot.arm.elbowMove(-2376);
        sleep(400);
        robot.arm.armMove(robot.arm.ARM_OUT_HIGH);
        sleep(1500);
        //robot.arm.wristMove(Arm.intakeStates.OFF);
        robot.forward(7);
        robot.arm.setSpecimen(false);
        sleep(2000);
        robot.back(28);
        robot.arm.armMove(0);
        sleep(1000);
        robot.arm.elbowMove(0);
        robot.turn(-65);
        robot.back(82);
        robot.strafeLeft(30);
    }
}
