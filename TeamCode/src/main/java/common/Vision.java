/*
 * This file contains support for TensorFlow object recognition and AprilTag recognition
 */
package common;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import test.code.LimelightTest;

public class Vision {
    //all calibration files are preset on the limelight, ignore controls for now
    Limelight3A limelight;

    private LinearOpMode opMode;

    public enum pipelines { RED, BLUE, YELLOW }

    public Vision(LinearOpMode opMode) {
        this.opMode = opMode;
        init();
    }

    public void init() {
        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");

        opMode.telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
    }

    public void limelightState(boolean active, int pipeline) {
        limelight.pipelineSwitch(pipeline);
        if (active) {
            limelight.start();
        } else {
            limelight.stop();
        }
    }

    //public void getResult()

}
