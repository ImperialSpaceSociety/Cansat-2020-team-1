
import processing.serial.*;
import grafica.*;

Serial SerialPort;
int NewLineChar = 10;          // ASCII for new line
String SerialBuffer = null;    // Buffer for serial data
float mX, mY, mZ;

int MaxReading = 100;

public GPlot plot1, plot2, plot3, plot4;

public void setup() {
  size(1000, 1000);
  
  String COMPort = Serial.list() [1];    // Create COM port
  SerialPort = new Serial(this, COMPort, 115200);  // Create Serial port
  
  // Setup for the first plot
    plot1 = new GPlot(this);
    plot1.setPos(0  , 0);
    plot1.setDim(400, 400);
    plot1.setXLim(-MaxReading, MaxReading);
    plot1.setYLim(-MaxReading, MaxReading);
    plot1.setHorizontalAxesTicksSeparation(10);
    plot1.setVerticalAxesTicksSeparation(10);
    plot1.getTitle().setText("mXY");
    plot1.getXAxis().getAxisLabel().setText("mX");
    plot1.getYAxis().getAxisLabel().setText("mY");
  
  // Setup for the second plot 
    plot2 = new GPlot(this);
    plot2.setPos(500, 0);
    plot2.setDim(400, 400);
    plot2.setXLim(-MaxReading, MaxReading);
    plot2.setYLim(-MaxReading, MaxReading);
    plot2.setHorizontalAxesTicksSeparation(10);
    plot2.setVerticalAxesTicksSeparation(10);
    plot2.getTitle().setText("mXZ");
    plot2.getXAxis().getAxisLabel().setText("mX");
    plot2.getYAxis().getAxisLabel().setText("mZ");
    
  // Setup for the third plot 
    plot3 = new GPlot(this);
    plot3.setPos(0, 500);
    plot3.setDim(400, 400);
    plot3.setXLim(-MaxReading, MaxReading);
    plot3.setYLim(-MaxReading, MaxReading);
    plot3.setHorizontalAxesTicksSeparation(10);
    plot3.setVerticalAxesTicksSeparation(10);
    plot3.getTitle().setText("mYZ");
    plot3.getXAxis().getAxisLabel().setText("mY");
    plot3.getYAxis().getAxisLabel().setText("mZ");
  
}

public void draw() {
  while(SerialPort.available()>0){    // If serial port open
    SerialBuffer = SerialPort.readStringUntil(NewLineChar);    // Read until new line
    
    if(SerialBuffer != null){    // If there is data
      int Comma1 = SerialBuffer.indexOf(",");            // Find first instance of comma
      int Comma2 = SerialBuffer.indexOf(",",Comma1+1);   // Second instance of comma      
      String mXStr = SerialBuffer.substring(0,Comma1);          // Split strings
      String mYStr = SerialBuffer.substring(Comma1+1,Comma2);
      String mZStr = SerialBuffer.substring(Comma2+1);      
      mX = float(mXStr);    // Convert to float
      mY = float(mYStr);
      mZ = float(mZStr);
    
      background(255);
      
      plot1.addPoint(mX,mY);
      plot2.addPoint(mX,mZ);
      plot3.addPoint(mY,mZ);
      
      // Draw plot 1
        plot1.beginDraw();
        plot1.drawBackground();
        plot1.drawBox();
        plot1.drawXAxis();
        plot1.drawYAxis();
        plot1.drawTitle();
        plot1.drawGridLines(GPlot.BOTH);
        plot1.drawPoints();
        plot1.endDraw();
      // Draw plot 2 
        plot2.beginDraw();
        plot2.drawBackground();
        plot2.drawBox();
        plot2.drawXAxis();
        plot2.drawYAxis();
        plot2.drawTitle();
        plot2.drawGridLines(GPlot.BOTH);
        plot2.drawPoints();
        plot2.endDraw();
      // Draw plot 3 
        plot3.beginDraw();
        plot3.drawBackground();
        plot3.drawBox();
        plot3.drawXAxis();
        plot3.drawYAxis();
        plot3.drawTitle();
        plot3.drawGridLines(GPlot.BOTH);
        plot3.drawPoints();
        plot3.endDraw();
    }
  }
}
