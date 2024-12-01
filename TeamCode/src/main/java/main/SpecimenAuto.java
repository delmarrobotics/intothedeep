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

@Autonomous(name="Specimen Auto", group="Main")
public class SpecimenAuto extends LinearOpMode {

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
        robot.arm.setSpecimen(true);
        robot.arm.elbowMove(robot.arm.ELBOW_RUNG);
        robot.arm.armMove(robot.arm.ARM_RUNG);
        robot.arm.wristMove(Arm.intakeStates.REVERSE);
        sleep(1000);
        robot.forward(30);
        robot.forwardSlow(4);
        sleep(200);
        robot.arm.elbowMove(-1900);
        sleep(1900);
        robot.arm.wristMove(Arm.intakeStates.OFF);
        robot.arm.setSpecimen(false);
        robot.back(10);
        robot.arm.elbowMove(0);
        robot.arm.armMove(0);
        robot.strafeRight(46.5);
        robot.forward(36);
        robot.strafeRight(10);
        robot.backFast(44);
        robot.forwardFast(48);
        robot.strafeRight(12);
        robot.turn(-15);
        robot.backFast(44);
        robot.forwardFast(44);
        robot.strafeRight(10);
        robot.backFast(44);
        robot.forwardFast(14);
        robot.turn(190);
        robot.arm.elbowMove(Arm.ELBOW_SAMPLE);
        robot.arm.armMove(Arm.ARM_SAMPLE);
        robot.arm.setSpecimen(false);
        robot.arm.wristMove(Arm.intakeStates.REVERSE);
        robot.strafeLeft(5);
        robot.forward(30);
        robot.arm.setSpecimen(true);
        robot.arm.wristMove(Arm.intakeStates.OFF);
        sleep(100);
        robot.arm.elbowMove(-600);
        robot.back(10);
        robot.arm.elbowMove(Arm.ELBOW_RUNG);
        robot.arm.armMove(Arm.ARM_RUNG);
        robot.turn(-100);
        robot.forward(38);
        robot.turn(-100);
    }
}
