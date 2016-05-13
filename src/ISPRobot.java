import java.net.URI;

import io.socket.client.Manager;
import io.socket.client.Socket;
import io.socket.emitter.Emitter;
import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

public class ISPRobot {

    static private Brick brick = BrickFinder.getDefault();
    static private GraphicsLCD g = brick.getGraphicsLCD();

    // Motors
    static private EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.B);
    static private EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.C);
    static private EV3LargeRegulatedMotor armMotor = new EV3LargeRegulatedMotor(MotorPort.A); // The port and "large" are both probably wrong

    // Sensors
    static private EV3GyroSensor gyro = new EV3GyroSensor(SensorPort.S2);
    static private EV3UltrasonicSensor sensor = new EV3UltrasonicSensor(SensorPort.S4);
    static private SampleProvider dist = sensor.getDistanceMode();
    static private EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S3);
    static private SampleProvider color = colorSensor.getRGBMode();

    // Socket
    static private Socket socket;

    public static void main(String[] args) throws Exception {
        URI uri = new URI("http://192.168.14.222");
        Manager manager = new Manager(uri);
        socket = manager.socket("/robot");

        RegulatedMotor[] list = new RegulatedMotor[1];
        list[0] = rightMotor;

        leftMotor.synchronizeWith(list);

        print("[STARTUP] Initialized");

        g.drawString("Press any button", 0, 0, GraphicsLCD.VCENTER | GraphicsLCD.LEFT);
        Button.waitForAnyPress();
        g.clear();

        print("[DEBUG] Started");

        socket.on("emergency", new Emitter.Listener() {
            @Override
            public void call(Object... args) {
                Runtime.getRuntime().exit(0);
            }
        }).on("led-color", new Emitter.Listener() {
            @Override
            public void call(Object... args) {
                g.clear();
                Button.LEDPattern((int) args[0]);
            }
        }).on("command", new Emitter.Listener() {
            @Override
            public void call(Object... args) {
                String direction = (String) args[0];
                switch (direction) {
                    case "forward":
                        print("[DEBUG] Moving forward");
                        setForward();
                        break;
                    case "backward":
                        print("[DEBUG] Moving backward");
                        setBackward();
                        break;
                    case "stop":
                        print("[DEBUG] Stopping");
                        stop();
                        break;
                    case "right":
                        print("[DEBUG] Turning right");
                        continuousTurnRight();
                        break;
                    case "left":
                        print("[DEBUG] Turning left");
                        continuousTurnLeft();
                        break;
                    case "raisearm":
                        print("[DEBUG] Raising arm");
                        raiseArm();
                        break;
                    case "lowerarm":
                        print("[DEBUG] Lowering arm");
                        lowerArm();
                        break;
                    default:
                        print("[ERROR] Received invalid movement command");
                        break;
                }
            }
        });
        socket.connect();

        Timer scheduler = new Timer(500, new TimerListener() {
            float[] colorSample = new float[color.sampleSize()];
            float[] distSample = new float[dist.sampleSize()];
            float[] gyroSample = new float[gyro.sampleSize()];
            @Override
            public void timedOut() {
                // TODO: Implement arm safety…somehow
                gyro.fetchSample(gyroSample, 0);
                dist.fetchSample(distSample, 0);
                color.fetchSample(colorSample, 0);
                socket.emit("gyro-sample", gyroSample[0]);
                socket.emit("infrared-sample", distSample[0]);
                socket.emit("color-sample", colorSample);
            }
        });
        scheduler.start();

        Timer safety = new Timer(5, new TimerListener() {
            float[] distSample = new float[dist.sampleSize()];
            @Override
            public void timedOut() {
                dist.fetchSample(distSample, 0);
                if (distSample[0] < 0.2) stop(); // Protects against (physical) crashing
            }
        });
        safety.start();

        // Close motors?
        g.clear();
        g.drawString("Executing…", 0, 0, GraphicsLCD.VCENTER | GraphicsLCD.LEFT);
    }

    private static void setForward() {
        leftMotor.startSynchronization();
        leftMotor.forward();
        rightMotor.forward();
        leftMotor.endSynchronization();
    }

    public static void setBackward() {
        leftMotor.startSynchronization();
        leftMotor.backward();
        rightMotor.backward();
        leftMotor.endSynchronization();
    }

    private static void raiseArm() {
        armMotor.forward();
    }

    private static void lowerArm() {
        armMotor.backward();
    }

    private static void stop() {
        leftMotor.startSynchronization();
        leftMotor.stop();
        rightMotor.stop();
        leftMotor.endSynchronization();
        armMotor.stop();
    }

    private static void continuousTurnRight() {
        rightMotor.stop();
        leftMotor.forward();
    }

    private static void continuousTurnLeft() {
        leftMotor.stop();
        rightMotor.forward();
    }


    private static void print(String message) {
        socket.emit("console-message", message);
        System.out.println(message);
    }

}
