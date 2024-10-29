/*
 * This OpMode calibrate any of the robot motors.
 */

/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package test.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.Arrays;
import java.util.Locale;
import java.util.SortedSet;

import common.Logger;

@TeleOp(name="Calibrate Arm", group="Test")
@SuppressWarnings("unused")

public class CalibrateArm extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    private final double incrementSlow = 1;
    private final double incrementMedium = 10;
    private final double incrementFast = 100;

    private final double speed = 1;
    private DcMotor left = null;
    private DcMotor right = null;

    private int home;
    private int target;

    @Override
    public void runOpMode() {

        telemetry.addLine("Press start");
        telemetry.update();
        telemetry.setAutoClear(false);

        home = 3; //ToDo fix stalling at 0 error
        target = 3565;

        left = hardwareMap.get(DcMotor.class, "leftArm");
        right = hardwareMap.get(DcMotor.class, "rightArm");

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        Telemetry.Item directionMsg = telemetry.addData("Motor direction", 0);
        Telemetry.Item positionMsg = telemetry.addData("Motor position", 0);
        Telemetry.Item homeMsg = telemetry.addData("Home position", 0);
        Telemetry.Item targetMsg = telemetry.addData("Target position", 0);

        setDisplayDirection(directionMsg);
        setDisplayPosition(positionMsg);
        setDisplayHome(homeMsg);
        setDisplayTarget(targetMsg);

        telemetry.addData("\nMotor Calibration Controls", "\n" +
                "  left trigger - run motor backwards\n" +
                "  right trigger - run the motor forward\n" +
                "  left stick - increase/decrease target position\n" +
                "  right stick - increase/decrease home position\n" +
                "  y - set home position to current position\n" +
                "  a - set target position to current position\n" +
                "  x - run to home position\n" +
                "  b - run motor to target position\n" +
                "  back - exit without saving\n" +
                "\n");

        telemetry.update();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                // set target position to current position
                target = left.getCurrentPosition();

            } else if (gamepad1.y) {
                // set the home position to the current position
                home = left.getCurrentPosition();

            } else if (gamepad1.x) {
                // run to home position
                runToPosition(home);

            } else if (gamepad1.b) {
                // run motor to an target position
                runToPosition(target);

            } else if (gamepad1.left_trigger > 0) {
                // manually run the motor backwards
                left.setPower(-speed);
                right.setPower(-speed);
                while (gamepad1.left_trigger > 0) {
                    setDisplayPosition(positionMsg);
                    telemetry.update();
                    sleep(100);
                }
                left.setPower(0);
                right.setPower(0);

            } else if (gamepad1.right_trigger > 0) {
                // manually run the motor forward
                left.setPower(speed);
                right.setPower(speed);
                while (gamepad1.right_trigger > 0) {
                    setDisplayPosition(positionMsg);
                    telemetry.update();
                    sleep(100);
                }
                left.setPower(0);
                right.setPower(0);

            } else if (gamepad1.left_stick_y > 0) {
                // increase target position
                runtime.reset();
                while (gamepad1.left_stick_y > 0) {
                    target -= increment(incrementSlow, incrementMedium, incrementFast);
                    setDisplayTarget(targetMsg);
                    telemetry.update();
                }

            } else if (gamepad1.left_stick_y < 0) {
                // decrease the target position
                runtime.reset();
                while (gamepad1.left_stick_y < 0) {
                    target += increment(incrementSlow, incrementMedium, incrementFast);
                    setDisplayTarget(targetMsg);
                    telemetry.update();
                }
                left.setPower(0);
                right.setPower(0);

            } else if (gamepad1.right_stick_y > 0) {
                // increase home position
                runtime.reset();
                while (gamepad1.right_stick_y > 0) {
                    home -= increment(incrementSlow, incrementMedium, incrementFast);
                    setDisplayHome(homeMsg);
                    telemetry.update();
                }

            } else if (gamepad1.right_stick_y < 0) {
                // decrease the home position
                runtime.reset();
                while (gamepad1.right_stick_y < 0) {
                    home += increment(incrementSlow, incrementMedium, incrementFast);
                    setDisplayHome(homeMsg);
                    telemetry.update();
                }
                left.setPower(0);
                right.setPower(0);


            } else if (gamepad1.dpad_up) {
                // change the direction of the motor
                if (left.getDirection() == DcMotor.Direction.FORWARD) {
                    left.setDirection(DcMotor.Direction.REVERSE);
                    right.setDirection(DcMotor.Direction.FORWARD);
                } else {
                    left.setDirection(DcMotor.Direction.FORWARD);
                    right.setDirection(DcMotor.Direction.REVERSE);
                }
                setDisplayDirection(directionMsg);
                while (gamepad1.dpad_up) {
                    sleep(10);
                }

            } else if (gamepad1.back) {
                // exit opmode without saving calibration data
                requestOpModeStop();
                break;
            }

            setDisplayPosition(positionMsg);
            setDisplayHome(homeMsg);
            setDisplayTarget(targetMsg);
            telemetry.update();
        }
    }

    private void setDisplayPosition(Telemetry.Item item) {
        item.setValue("%d", left.getCurrentPosition());
    }

    private void setDisplayHome(Telemetry.Item item) {
        item.setValue("%d", home);
    }

    private void setDisplayTarget(Telemetry.Item item) {
        item.setValue("%d", target);
    }

    private void setDisplayDirection(Telemetry.Item item) {
        item.setValue("%s", left.getDirection());
    }

    /**
     * Based on the elapsed time return a value to increment by
     *
     * @return value to increment by
     */
    public double increment(double v1, double v2, double v3) {
        int sleepTime;
        double delta;
        if (runtime.seconds() < 3) {
            delta = v1;
            sleepTime = 500;
        } else if (runtime.seconds() < 6) {
            delta = v2;
            sleepTime = 500;
        } else if (runtime.seconds() < 9) {
            delta = v3;
            sleepTime = 500;
        } else {
            delta = v3;
            sleepTime = 250;
        }

        sleep(sleepTime);
        return delta;
    }

    private void runToPosition(int position) {

        //DcMotor.RunMode mode = motor.getMode();
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);   
        left.setTargetPosition(position);
        right.setTargetPosition(position);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setPower(speed);
        right.setPower(speed);
        while (opModeIsActive()) {
            if (!left.isBusy() || gamepad1.left_bumper)
                break;
        }
        left.setPower(0);
        right.setPower(0);
    }
}