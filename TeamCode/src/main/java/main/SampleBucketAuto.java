/*
 * This file contains a OpMode for the autonomous phase when the robot starts at
 * the blue observation position.
 *
 */

package main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/*import org.checkerframework.checker.propkey.qual.PropertyKeyBottom;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;*/

//import common.Auto;
import common.Arm;
import common.Robot;

@Autonomous(name="Sample Bucket Auto", group="Main")
public class SampleBucketAuto extends LinearOpMode {

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
        robot.strafeRight(25.5);
        robot.arm.elbowMove(-2376);
        sleep(500);
        robot.arm.armMove(robot.arm.ARM_OUT_HIGH);
        sleep(2500);
        robot.forward(27);
        //sleep(750);
        robot.arm.intakeSet(Arm.intakeStates.REVERSE);
        sleep(1500);
        robot.arm.intakeSet(Arm.intakeStates.OFF);
        robot.back(27);
        robot.arm.elbowMove(0);
        robot.arm.armMove(0);
        sleep(1500);
        robot.turn(-51);
        robot.back(4);
        robot.strafeRight(13);
        robot.arm.wristMove(Arm.intakeStates.FORWARD);
        robot.arm.intakeSet(Arm.intakeStates.FORWARD);
        robot.arm.armMove(-1244);
        robot.arm.elbowMove(200);
        sleep(200);
        robot.arm.wristMove(Arm.intakeStates.OFF);
        sleep(1800);
        robot.arm.elbowMove(0);
        robot.arm.armMove(0);
        robot.strafeLeft(13);
        robot.turn(51);
        robot.arm.wristMove(Arm.intakeStates.REVERSE);
        robot.arm.elbowMove(-2376);
        sleep(200);
        robot.arm.wristMove(Arm.intakeStates.OFF);
        robot.arm.armMove(robot.arm.ARM_OUT_HIGH);
        sleep(2000);
        robot.forward(27);
        robot.arm.intakeSet(Arm.intakeStates.REVERSE);
        sleep(2000);
        robot.arm.intakeSet(Arm.intakeStates.OFF);
        robot.back(27);
        robot.arm.elbowMove(0);
        robot.arm.armMove(0);
        robot.turn(-51);
        robot.back(82);
        robot.strafeLeft(30);*/
    }
}
