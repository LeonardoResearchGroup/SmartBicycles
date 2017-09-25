import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.UnknownHostException;

public class ComTCP extends Thread {

  private int puerto;
  ServerSocket server;
  private Socket cliente;
  private String message;
  private int estado;

  public ComTCP() {
    puerto = 5000;
    estado=0;
    try {
      System.out.println("ComTCP()> Correr app de Unity");
      server = new ServerSocket(puerto);
      cliente = server.accept();
      System.out.println("ComTCP()> Conectado exitosamente :)");
    } 
    catch (IOException e) {
      e.printStackTrace();
    }
    start();
  }

  public void run() {
    while (true) {
      System.out.println("...");
      System.out.println("ComTCP().run() Recibiendo de Unity");
      recibir();

      try {
        System.out.println("ComTCP().run() hilo servidor durmiendo");
        sleep(10);
      } 
      catch (InterruptedException e) {
        e.printStackTrace();
      }
      
    }
  }

  public void recibir() {
    InputStream entradaBytes;
    DataInputStream entradaDatos;
    try {
      entradaBytes = cliente.getInputStream();
      entradaDatos = new DataInputStream(entradaBytes);
      byte[] temp = new byte[64000];
      String mensaje = entradaDatos.readLine();
      imprimirMensajeUnity(mensaje);
      message=mensaje;
      estado= Integer.parseInt(message);
    } 
    catch (IOException e) {
      e.printStackTrace();
    }
  }


  public void enviar(String msj) {
    OutputStream salidaBytes;
    DataOutputStream salidaDatos;
    try {
      salidaBytes = cliente.getOutputStream();
      salidaDatos = new DataOutputStream(salidaBytes);
      //salidaDatos.writeUTF(msj);
      byte[] paquete = msj.getBytes();
      salidaDatos.write(paquete);
      System.out.println("ComTCP().enviar()> Mensaje enviado a Unity: " + msj);
      salidaDatos.flush();
    } 
    catch (IOException e) {
      e.printStackTrace();
    }
  }
  public int getEstado() {
    return estado;
  }

  public void imprimirMensajeUnity(String valor) {
    String mensaje = "ComTCP().imprimirMensajeUnity()> mensaje vacio";
    if (valor.equals("-1")) {
      mensaje= " decrease";
    } else if (valor.equals("1")) {
      mensaje = " increase";
    } else if (valor.equals("0")) {
      mensaje = " in green wave";
    }
    println("ComTCP().imprimirMensajeUnity()> : valor recibido:" +valor + " "+ mensaje);
  }
}