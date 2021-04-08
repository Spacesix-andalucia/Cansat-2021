#include <SoftwareSerial.h>

SoftwareSerial radio_enlace(10, 11); 

void setup() {

  //Activa la alimentacion de la antena en su conexion al Arduino Uno
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  //Inicializa el puerto serie entre el Arduino Uno y el ordenador
  Serial.begin(9600);
  while (!Serial) {
    ; 
  }
  Serial.println("Estacion Base Activada y lista para recibir datos y enviar ordenes al Cansat ");
  Serial.println("=============================================================================");
  Serial.println("Escribir L para realizar el lanzamiento");
  Serial.println("Escribir D para recuperar los datos del anterior lanzamiento");
  Serial.println("");
  Serial.println("");

  //Inicializa el radioenlace entre el Arduino Uno de la estacion Base 
  // y el arduino Nano del Cansat
  radio_enlace.begin(9600);
  radio_enlace.println("Mensaje enviado desde la estacion base al Cansat de Space Six");
}

void loop() { 
  // Escribe en el monitor serial los datos recibidos por el Radio Enlace
  while (radio_enlace.available()) {
    Serial.write(radio_enlace.read());
  }

  // Manda por radio las ordenes escritas en el terminal de la estacion base
  while (Serial.available()) {
    radio_enlace.write(Serial.read());
  }
}
