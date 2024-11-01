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
        robot.strafeRight(25.5);
        robot.arm.elbowMove(-3224);
        sleep(500);
        robot.arm.armMove(robot.arm.ARM_OUT_HIGH);
        sleep(3000);
        robot.forward(26);
        robot.arm.intakeSet(Arm.intakeStates.REVERSE);
        sleep(1500);
        robot.arm.intakeSet(Arm.intakeStates.OFF);
        robot.back(25);
        robot.arm.elbowMove(0);
        robot.arm.armMove(0);
        sleep(3000);
        robot.turn(-48);
        robot.back(82);
        robot.strafeLeft(30);
    }
}
