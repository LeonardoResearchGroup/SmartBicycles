import processing.serial.*;

private Serial myPort;  // Create object from Serial class


private ComTCP com;
private int ultimoEstado=0;
private int vel =0;
private String val;      // Data received from the serial port
private String ultimaVel="";
private String ultimaVelKeyboard="";
private String mensajeNoData="";
private int mapVel;

//private int vel=18; //debuggin
void setup() 
{
  size(200, 200);
  //Look for the Bluetooth port must be something like tty.HC-6 on MAC
  for (int i=0; i<Serial.list ().length; i++) {
    println(Serial.list()[i]);
  }
  String portName = Serial.list()[2];
  println("Selected port: "+portName);
  myPort = new Serial(this, portName, 9600);
  com=new ComTCP();
}

void draw(){
  background(255);
  if ( myPort.available() > 0) {  // If data is available,
    mensajeNoData = "Datos frescos en puerto serial";
    val = myPort.readStringUntil(35);        // read it and store it in val
    //println("recibi: "+val);
    if (ultimaVel!=null && val!=null) {
      // Removes input string splitter
      String tempVel= val.replaceAll("#", "");
      println("TempVel: " + tempVel);
      try {
        // Converts input string to integer
        mapVel= floor(Float.parseFloat(tempVel));
      }
      catch(NumberFormatException e) {
        println("Wrong number format");
      }
    }
  } else if (!mensajeNoData.equals("*** No hay datos frescos en puerto serial")) {
    mensajeNoData = "*** No hay datos frescos en puerto serial";
    println(mensajeNoData);
  }

  /* Send the latest speed to the Unity Simulator
   If the number is different than previous
   */
  if (!ultimaVel.equals(mapVel+"")) {
    println("Enviar vel diferente: " + mapVel);
    com.enviar(mapVel+"");
    ultimaVel=(mapVel+"");
  }

  //---------for debuggin----------//
  if (ultimaVelKeyboard!=null) {
    if (!ultimaVelKeyboard.equals(vel+"")) {
      com.enviar(vel+"");
      ultimaVelKeyboard=(vel+"");
    }
  }
  //////////////////////////////////

  if (ultimoEstado!=com.getEstado()) {
    myPort.write(com.getEstado()+"#");
    ultimoEstado=com.getEstado();
    //println("envie a Arduino: "+ com.getEstado()+"#");
  }
   text("Vel keyboard: "+vel, 50, 50);
  text("Vel joystick: "+mapVel, 50, 80);
}

void keyPressed() {
   if (key == CODED) {
    if (keyCode == UP) {
      vel+=1;
    } else if (keyCode == DOWN) {
      vel-=1;
    } 
  }
}

