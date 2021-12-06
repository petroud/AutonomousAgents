// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import com.cyberbotics.webots.controller.Accelerometer;
import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.LightSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

import java.util.Random;

public class Rat1 extends Robot {

  protected final int timeStep = 32;
  protected final double maxSpeed = 300;
  protected final double[] collisionAvoidanceWeights = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  protected final double[] slowMotionWeights = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

  protected Accelerometer accelerometer;
  protected Camera camera;
  protected int cameraWidth, cameraHeight;
  protected Motor leftMotor, rightMotor;
  protected DistanceSensor[] distanceSensors = new DistanceSensor[8];
  protected LightSensor[] lightSensors = new LightSensor[8];
  protected LED[] leds = new LED[10];

  public Rat1() {
    accelerometer = getAccelerometer("accelerometer");
    camera = getCamera("camera");
    camera.enable(8*timeStep);
    cameraWidth=camera.getWidth();
    cameraHeight=camera.getHeight();
    leftMotor = getMotor("left wheel motor");
    rightMotor = getMotor("right wheel motor");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);
    leftMotor.setVelocity(0.0);
    rightMotor.setVelocity(0.0);
    for (int i=0;i<10;i++) {
      leds[i]=getLED("led"+i);
    };
    for (int i=0;i<8;i++) {
      distanceSensors[i] = getDistanceSensor("ps"+i);
      distanceSensors[i].enable(timeStep);
      lightSensors[i] = getLightSensor("ls"+i);
      lightSensors[i].enable(timeStep);
    }
    batterySensorEnable(timeStep);
  }

  public void run() {

    int blink = 0;
    int oldDx = 0;
    Random r = new Random();
    boolean turn = false;
    boolean right = false;
    boolean seeFeeder = false;
    double battery;
    double oldBattery = -1.0;
    int image[];
    double distance[] = new double[8];
    int ledValue[] = new int[10];
    double leftSpeed, rightSpeed;

   while (step(timeStep) != -1) {

      // read distance & battery sensor information
      for(int i=0;i<8;i++) distance[i] = distanceSensors[i].getValue();
      battery = batterySensorGetValue();
      
      //Distance from the right wall to follow is OK
      boolean right_wall = distance[5] > 310;
      
      //Detected an obstacle in front with one of the 2 back sensors
      boolean obstacle_front = distance[4] > 400 || distance[3] > 400;
      
      //The left hand side back sensor (right front on the direction of backwards movement) detected something
      //This means the robot is doing a 180 right turn and needs to adjust its angle parallel to the wall
      boolean right_corner = distance[4] > 200;
      
      //Initially set the two motors at max speed
      leftSpeed = maxSpeed;
      rightSpeed = maxSpeed;
      
      //Obstacle detected in front 
      if(obstacle_front){
        //System.out.println("OMG an obstacle in front!");
        //Turn right (dir of movement) to avoid it and to obey to the R.W.F. rule
        leftSpeed = maxSpeed;
        rightSpeed = -maxSpeed;
      }else{ 
        //The distance from the right hand side wall is okay, i obey to the RWF rule continue straight
        if(right_wall){
          //System.out.println("Driving forward...");
          leftSpeed = maxSpeed;
          rightSpeed = maxSpeed;
        }else{
          //I lost the wall on the right, let me turn right to find the next one closest
          //System.out.println("Lost the right wall...");
          leftSpeed = maxSpeed/8;
          rightSpeed = maxSpeed;
        }
        
        //I had an 180 right turn, let me fix my position parallel to the wall to avoid crashing to it
        if(right_corner){
          //System.out.println("Oh I came too close");
          leftSpeed = maxSpeed;
          rightSpeed = maxSpeed/2.5;
        }
      }
      
    
      //recharging behavior
      if (battery > oldBattery) {
        leftSpeed  = 0.0;
        rightSpeed = 0.0;
      }
      oldBattery = battery;
      if (blink++ >= 20) { // blink the back LEDs
        if (blink == 40) blink = 0;
      }
      
      //Reverse all the way
      leftMotor.setVelocity(-0.00628 * leftSpeed);
      rightMotor.setVelocity(-0.00628 * rightSpeed);
    } 
    // Enter here exit cleanup code
  }

  public static void main(String[] args) {
    Rat1 rat1 = new Rat1();
    rat1.run();
  }
}
