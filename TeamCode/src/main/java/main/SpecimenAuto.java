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
        robot.arm.elbowMove(robot.arm.ELBOW_RUNG);
        robot.arm.armMove(robot.arm.ARM_RUNG);
        robot.forward(34);
        robot.arm.wristMove(Arm.intakeStates.REVERSE);
        sleep(3000);
        robot.arm.elbowMove(-1900);
        sleep(2000);
        robot.arm.wristMove(Arm.intakeStates.OFF);
    }
}
