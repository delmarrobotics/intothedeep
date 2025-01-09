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
        /*paused for new arm
        robot.arm.setSpecimen(true);
        robot.arm.elbowMove(-2100);
        robot.arm.armMove(-1500);
        robot.arm.wristMove(Arm.intakeStates.REVERSE);
        sleep(1000);
        robot.forward(30);
        robot.forwardSlow(4);
        sleep(200);
        robot.arm.elbowMove(-1950);
        sleep(1950);
        robot.arm.wristMove(Arm.intakeStates.OFF);
        robot.arm.setSpecimen(false);
        robot.back(10);
        robot.arm.elbowMove(0);
        robot.arm.armMove(0);
        robot.strafeRight(43);
        robot.forward(36);
        robot.strafeRight(10);
        robot.backFast(44);
        robot.forwardFast(48);
        robot.strafeRight(16);
        robot.turn(-15);
        robot.backFast(44);
        robot.forwardFast(44);
        robot.strafeRight(10);
        robot.backFast(44);
        robot.forward(14);
        /*robot.turn(180);
        robot.arm.elbowMove(Arm.ELBOW_SAMPLE);
        robot.arm.armMove(Arm.ARM_SAMPLE);
        robot.arm.wristMove(Arm.intakeStates.FORWARD);
        sleep(50);
        robot.arm.setSpecimen(false);
        robot.strafeLeft(30);
        sleep(100);
        robot.forward(23);
        robot.arm.setSpecimen(true);
        robot.forwardSlow(3);
        robot.arm.wristMove(Arm.intakeStates.OFF);
        robot.arm.elbowMove(800);
        robot.back(10);
        sleep(50);
        robot.arm.elbowMove(Arm.ELBOW_RUNG);
        robot.arm.armMove(Arm.ARM_RUNG);
        sleep(50);
        robot.turn(-110);
        robot.forward(43);
        robot.turn(-110);
        robot.arm.elbowMove(-2100);
        robot.arm.armMove(-1500);
        robot.forward(10);
        robot.forwardSlow(4);
        sleep(200);
        robot.arm.elbowMove(-1950);
        sleep(1950);
        robot.arm.wristMove(Arm.intakeStates.OFF);
        robot.arm.setSpecimen(false);
        robot.back(10);
        robot.arm.elbowMove(0);
        robot.arm.armMove(0);*/
        /*sleep(500);
        robot.strafeRight(25);
        robot.back(20);*/
    }
}
