/*
 * This file contains a OpMode for the autonomous phase when the robot starts at
 * the blue observation position.
 *
 */

package main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import common.Robot;

@Autonomous(name="Specimen Auto 3", group="Main")
public class SpecimenAuto3 extends LinearOpMode {

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
        robot.arm.setSpecimenLeft(true);
        //sleep(14000);
        //robot.strafeRight(11);
        robot.arm.elbowMoveLeft(-1260);
        robot.arm.armMoveLeft(-1500);
        //sleep(1000);
        robot.forward(37);
        //robot.back(1);
        //robot.back(1.5);
        //sleep(200);
        robot.arm.elbowMoveLeft(-450);
        sleep(800);
        robot.arm.setSpecimenLeft(false);
        robot.back(12);
        robot.arm.elbowMoveLeft(24);
        robot.arm.armMoveLeft(25);
        robot.strafeRight(46);
        robot.forward(47);
        robot.turn(180);
        robot.strafeLeft(16);
        robot.forward(51);
        robot.strafeLeft(30);
        robot.forward(20);
        robot.arm.setSpecimenLeft(true);
        sleep(200);
        robot.back(14);
        robot.strafeRight(22);
        robot.turn(-90);
        robot.back(45);
        robot.arm.elbowMoveLeft(-1260);
        robot.arm.armMoveLeft(-1500);
        robot.turn(-90);
        robot.forward(20);
        robot.arm.elbowMoveLeft(-450);
        sleep(900);
        robot.arm.setSpecimenLeft(false);
        robot.back(14);
        robot.turn(90);
        robot.arm.elbowMoveLeft(24);
        robot.arm.armMoveLeft(25);
        robot.forward(59);
        robot.turn(90);
        robot.strafeLeft(17);
        robot.forward(23.5);
        robot.arm.setSpecimenLeft(true);
        sleep(100);
        robot.back(14);
        robot.strafeRight(22);
        robot.turn(-90);
        robot.back(47);
        robot.arm.elbowMoveLeft(-1260);
        robot.arm.armMoveLeft(-1500);
        robot.turn(-90);
        robot.forward(20);
        robot.arm.elbowMoveLeft(-450);
        sleep(600);
        robot.arm.setSpecimenLeft(false);
        robot.back(12);
        robot.arm.elbowMoveLeft(0);
        robot.back(20);
        robot.strafeRight(61);

        /*robot.arm.elbowMoveLeft(0);
        robot.arm.armMoveLeft(0);
        robot.strafeRight(38);
        robot.forward(29);
        robot.strafeRight(9);
        robot.backFast(39);
        robot.forwardFast(39);
        robot.strafeRight(12);*/
    }
}
