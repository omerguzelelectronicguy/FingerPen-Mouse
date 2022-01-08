import processing.serial.*;
import java.awt.Robot;

Robot rbt;


Serial myPort;
String val;

void setup() 
{
  String portName = "COM7";
  myPort = new Serial(this, portName, 115200);
  try {
    rbt = new Robot();
  } catch(Exception e) {
    e.printStackTrace();
  }
}
int posx=0;
int posy=0;
void draw()
{
  if ( myPort.available() > 4) { 
    byte[] a = new byte[8];    
    
    myPort.readBytes(a);
    int x = a[2];
    int y = a[4];
    int b = x ;
    //println(x +"\t" + y);    
    posx = posx + x;
    posy = posy + y;
    rbt.mouseMove(posx,posy);
  }
  
}
