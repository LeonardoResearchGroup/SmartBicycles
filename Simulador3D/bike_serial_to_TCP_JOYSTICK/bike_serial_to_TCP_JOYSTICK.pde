/*
Este prototipo esta constituido de 3 compunentes: i) una interfaz fisica que va instalada en la bicileta
ii) un servidor y iii) una aplicación que simula el desplazamiento de una bicicleta en la ciudad. 

La interfaz fisica esta controlada por un arduino que tiene conectados un sensor de velocidad y 
tres actuadores: un led rojo, uno verde y un motor bidireccional de rotacion continua. 

El arduino envia la velocidad unicamente cuando hay un cambio. De esta forma el canal solo se ocupa cuando 
hay cambios que notificar. El servidor lee el puerto serial solamente cuando se produce un evento, es decir
que lo lee solo cuando hay una señal enviada desde el arduino. Cuando lee la señal la convierte en un numero
flotante y lo envia al simulador a través del servidor. De acuerdo al valor recibido el simulador acelera o
desacelera la bicicleta. El acelerador esta constantemente detectando si la bicicleta esta dentro de una ola
verde. Cuando la sobrepasa envia una señal de -1 que significa desaceleracion. Cuando se rezaga manda una 
señal positiva de aceleración y finalmente cuando esta en la ola verde manda una señal igual a 0 que indica 
que esta dentro de la ola verde. Esos datos los recibe el servidor y se los envia al arduino por el canal de
bluetooth. Dependiendo del valor arduino enciende los leds rojo para desacelerar, verde para acelerar o azul
para indicar que esta en la velocidad correcta. Con base en la misma señal hace girar el motor hacia adelante
cuando la señal es positiva y hacia atrás cuando es negativa.
*/

import processing.serial.*; 

Serial myPort;    // The serial port that connects to arduino
ComTCP com; // This is the server that connects to Unity App
PFont myFont;     // The display font
String inString;  // Input string from serial port
int lf = 35;      // ASCII linefeed 
int mapVel = 0;
int ultimoEstado= -3;
int ultimaVel = 0;
Bar bar;

void setup() { 
  size(100, displayHeight-100); 
  printArray(Serial.list()); 
  myPort = new Serial(this, Serial.list()[2], 9600); 
  myPort.bufferUntil(lf);
  com=new ComTCP();
  bar = new Bar(45);
  bar.setMinMax(0,30);
} 

void draw() { 
  background(0); 
  if (ultimaVel != mapVel) {
    com.enviar(mapVel+"");
    ultimaVel = mapVel;
    // This delay is critical to evacuate the Serial buffer
    delay(100);
  }
  // Estado bicy - ola verde
  if (ultimoEstado!=com.getEstado()) {
    myPort.write(com.getEstado()+"#");
    ultimoEstado=com.getEstado();
  }
  bar.show(ultimaVel);
} 

void serialEvent(Serial p) { 
  inString = p.readStringUntil(35); 
  inString = inString.replaceAll("#", "");
  try {
    // Converts input string to integer
    float tmp = map (Float.parseFloat(inString), 0, 20, 0, 30);
    mapVel= floor(tmp);
  }
  catch(NumberFormatException e) {
    println("Wrong number format");
  }
} 
