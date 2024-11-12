package test.code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import common.Config;

@TeleOp(name="Hello World")
@Disabled
public class HelloWorld extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addLine("Hello world!");
    }
}
