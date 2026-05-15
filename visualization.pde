import processing.serial.*;
import java.awt.event.KeyEvent;
import java.io.IOException;
Serial myPort;
String data="";
PShape craft;
float roll, pitch,yaw;

void setup() {
  printArray(Serial.list());
  size (1200, 800, P3D);
  surface.setResizable(true);
  craft = loadShape("plane.obj");

  myPort = new Serial(this, "COM5", 19200);
  myPort.bufferUntil('\n');
}

void draw() {
  translate(width/2, height/2, 0);
  background(11,11,30);
  textSize(24);
  text("Roll: " + int(roll) + "     Pitch: " + int(pitch) + "      Yaw: " + int(yaw%360), -100, 265);
  
  rotateX(radians(pitch));
  rotateZ(radians(roll));
  rotateY(radians(yaw));

  lights();
  ambientLight(60, 60, 80);
  directionalLight(255, 220, 180, -1, -1, -1);
  
  scale(0.1);

  rotateX(PI); //upside model fix
  
  shape(craft, 0, 0);
}

void serialEvent (Serial myPort) { 
  data = myPort.readStringUntil('\n');
  
  if (data != null) {
    data = trim(data);
    String items[] = split(data, '/');
    
    if (items.length > 1) {
        roll = float(items[0]);
        pitch = float(items[1]);
        yaw = float(items[2]);
    }
  }
  
}
