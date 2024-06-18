package frc.robot.subsystems;

import java.io.ByteArrayOutputStream;
import java.io.IOException;

import com.studica.frc.Lidar;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.models.CluePoint;

public class LidarTestOld extends SubsystemBase {
    private Lidar lidar;
    private Lidar.ScanData scanData;
    public boolean scanning = true;
    public LidarTestOld() {
        lidar = new Lidar(Lidar.Port.kUSB1);
    }

    public void startScan() {
        lidar.start();
        scanning = true;
    }
    public void stopScan() {
        lidar.stop();
        scanning = false;
    }

    public Lidar.ScanData getData() {
        return this.scanData;
    }

    public Mat generateLidarImage() {
        int width = 800;
        int height = 800;

        Mat image = Mat.zeros(height, width, CvType.CV_8UC3);
        Scalar color =  new Scalar(255, 255, 255);

        int originX = width / 2;
        int originY = height / 2;

        for (int i = 0; i < scanData.distance.length; i++) {
            double theta = Math.toRadians(scanData.angle[i]);
            double x = scanData.distance[i] * Math.cos(theta);
            double y = scanData.distance[i] * Math.sin(theta);

            int drawX = (int) (originX + x * 50);  // Scale factor of 50 for better visualization
            int drawY = (int) (originY - y * 50);  // Invert y-axis for correct orientation

            Imgproc.circle(image, new Point(drawX, drawY), 5, color, -1);
        }

        return image;

    }
    
    public void saveImage(Mat image, String path) {
        Imgcodecs.imwrite(path, image);
    }

    public byte[] matToByteArray(Mat mat) {
        MatOfByte baos = new MatOfByte();
        Imgcodecs.imencode(".png", mat, baos);
        return baos.toArray();
    }

    public void showPointsMap() {
        Mat image = this.generateLidarImage();
            
        byte[] imageData = this.matToByteArray(image);

        ShuffleboardTab tab = Shuffleboard.getTab("Lidar Sensor");
        NetworkTableEntry imageEntry = tab.add("Lidar Image", "").withWidget("Image").getEntry();

        imageEntry.setString(new String(imageData));
    }

    @Override
    public void periodic() {
        if(scanning) {
            scanData = lidar.getData();
            Mat image = this.generateLidarImage();
            
            byte[] imageData = this.matToByteArray(image);

            ShuffleboardTab tab = Shuffleboard.getTab("Lidar Sensor");
            NetworkTableEntry imageEntry = tab.add("Lidar Image", "").withWidget("Image").getEntry();

            imageEntry.setString(new String(imageData));
        }
    }
}