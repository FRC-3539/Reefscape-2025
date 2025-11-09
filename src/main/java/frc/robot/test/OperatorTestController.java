package frc.robot.test;

import java.util.ArrayList;

import org.json.simple.JSONObject;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedSubsystem;

public class OperatorTestController {
    public static Command operatorTestCommand;

    public static String currentExpectedInput = null;
    public static int correctPresses, incorrectPresses;
    public static ArrayList<Double> inputTimes = new ArrayList<Double>();

    public static Timer buttonTimer = new Timer();
    public static Timer overallTimer = new Timer();

    public OperatorTestController() {
        resetTest();
    }

    public static void expectInput(String button) {
        currentExpectedInput = button;
        buttonTimer.restart();
    }

    public static boolean processInput(String button) {
        if (button == currentExpectedInput) {
            buttonTimer.stop();
            inputTimes.add(buttonTimer.get());
            correctPresses++;
            currentExpectedInput = null;
            LedSubsystem.setLEDS(new Color(0, 0, 0));
            return true;
        }
        else {
            incorrectPresses++;
            return false;
        }
    }

    public static void resetTest() {
        correctPresses = 0;
        incorrectPresses = 0;
        inputTimes.clear();

        buttonTimer.reset();
        overallTimer.reset();
    }

    public static void startTest() {
        overallTimer.start();
    }

    public static void endTest() {
        overallTimer.stop();

        JSONObject results = new JSONObject();
        results.put("correctPresses", correctPresses);
        results.put("incorrectPresses", incorrectPresses);
        results.put("pressAccuracy", 1.0 * correctPresses / (correctPresses + incorrectPresses));
        
        double totalResponseTime = 0;
        for (double time : inputTimes) totalResponseTime += time;
        results.put("averageResponseTime", totalResponseTime / inputTimes.size());
        results.put("totalTime", overallTimer.get());

        SmartDashboard.putString("operatorTestResults", results.toJSONString());

        resetTest();
    }

    public static boolean testActive() {
        return overallTimer.isRunning();
    }
}
