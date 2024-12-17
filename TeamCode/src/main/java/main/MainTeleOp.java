package main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import common.Logger;
import common.Robot;

/*
 * This file contains the OpMode for phase two of the FTC Into The Deep Competition.
*/

@TeleOp(name="Main TeleOp", group="Main")
@SuppressWarnings("unused")
public class MainTeleOp extends LinearOpMode {

    public enum GamepadMode { ONE, TWO, THREE }
    public GamepadMode mode;
    public GamepadMode modeSplit;

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    // Initialize the robot hardware.
    private Robot robot = null;

    @Override
    public void runOpMode() {
        telemetry.addLine("Press start");
        telemetry.update();

        // Initialize the robot hardware.
        robot = new Robot(this);
        robot.init();

        robot.drive.setBraking(false);
        robot.drive.start();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        displayControls();
        robot.arm.displayControls();
        displayControls2();
        mode = GamepadMode.THREE;
        mode = GamepadMode.THREE;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // If the start button is pressed, ignore a and b buttons. Start button is used to
            // select active gamepad.
            if (gamepad1.start || gamepad2.start) {
                while (gamepad1.start || gamepad1.a || gamepad1.b || gamepad2.start || gamepad2.a || gamepad2.b)
                    sleep(100);
                continue;
            }
            // POV Mode uses left stick to go forward, and right stick to rotate, left trigger accelerate.
            /*
            double drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
            double strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
            double yaw   = -gamepad1.right_stick_x / 3.0;  // Reduce rotate rate to 33%.
            double speed = gamepad1.left_trigger;
            robot.moveRobot(drive, strafe, yaw, speed);
            */

            if (gamepad2.right_bumper) {
                Logger.message("Changed linkage at %f", runtime.seconds());
                changeLinkMode();
                while (gamepad2.right_bumper) sleep(100);
            }

            if (mode == GamepadMode.TWO) {
                robot.arm.specimenLeft.setPosition(gamepad1.right_trigger * 0.6 + 0.1);
                robot.arm.specimenRight.setPosition(gamepad2.right_trigger * 0.6 + 0.1);
                robot.arm.controlLeft();
                robot.arm.controlRight();
                if (gamepad1.right_bumper) {
                    Logger.message("Changed split mode (to THREE) at %f", runtime.seconds());
                    mode = GamepadMode.THREE;
                    modeSplit = GamepadMode.THREE;
                    while (gamepad1.right_bumper) sleep(100);
                }
            } else if (mode == GamepadMode.ONE) {
                /*if (robot.arm.positionCommand())
                    //robot.intakeOff();
                    telemetry.addData("intakeOff", "intakeOff");
                else if (robot.arm.dropCommand())
                    //robot.dropPixel();
                if (robot.arm.control())
                    continue;*/
                robot.arm.control();
            } else if (mode == GamepadMode.THREE) {
                robot.arm.specimenRight.setPosition(gamepad2.right_trigger * 0.6 + 0.1);
                robot.arm.controlRight();
                if (gamepad1.right_bumper) {
                    Logger.message("Changed split mode (to TWO) at %f", runtime.seconds());
                    mode = GamepadMode.TWO;
                    modeSplit = GamepadMode.TWO;
                    while (gamepad1.right_bumper) sleep(100);
                }
            }

            if (gamepad1.a) {
                while (gamepad1.a) sleep(100);

            } else if (gamepad1.y) {
                while (gamepad1.y) sleep(100);

            } else if (gamepad1.b) {;
                while (gamepad1.b) sleep(100);

            } else if (gamepad1.x) {
                while (gamepad1.x) sleep(100);

            } else if (gamepad1.right_bumper) {
                while (gamepad1.right_bumper) sleep(100);
            }

            if (gamepad1.back) {
                robot.leftHook.setPosition(-100);
                robot.rightHook.setPosition(100);
            }

            telemetry.update();
        }
    }

    private void changeLinkMode () {
        if (mode == GamepadMode.ONE) {
            mode = modeSplit;
        } else if (mode == GamepadMode.TWO || mode == GamepadMode.THREE) {
            mode = GamepadMode.ONE;
        }
        displayControls();
        displayControls2();
    }

    public void displayControls(){
        telemetry.addLine("Driver Controls (Gamepad 1)\n" +
                "  left stick - drive robot\n" +
                "  right stick - rotate robot\n" +
                "  left trigger - accelerate robot speed\n" +
                /*"  a - intake on / off\n" +
                "  b - intake rotate up / down\n" +
                "  y - intake reverse\n" +
                "  x - drive to backdrop\n" +*/
                "\n");
    }
    public void displayControls2() {
        if (mode == GamepadMode.TWO) {
            telemetry.addLine("linked mode");
        } else {
            telemetry.addLine("linked mode");
        }
        telemetry.addLine("\n  back - toggle gamepad2 (pixelArm / hangingArm)");
    }

}
