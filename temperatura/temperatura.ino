/* Sketch de ejemplo para testear el sensor de temperatura analógico TMP36
 Escrito por Regata para www.tallerarduino.wordpress.com
 
 MODO DE CONEXIÓN DEL SENSOR
 
 Conectamos el pin 1 que corresponde a la alimentación del sensor con los 5V del Arduino
 Conectamos el pin 2 que corresponde al pin de datos del sensor con cualquier pin analógico del Arduino
 Conectamos el pin 3 que corresponde al pin de masa (GND) del sensor con el pin GND del Arduino
 
*/
 
int temp = 5;  //Pin analógico A5 del Arduino donde conectaremos el pin de datos del sensor TMP36
float maxC = 0, minC = 100, maxF = 0, minF = 500, maxV = 0, minV = 5;  //Variables para ir comprobando maximos y minimos
 
void setup()
{
  Serial.begin(9600);  //Iniciamos comunicación serie con el Arduino para ver los resultados del sensor
                        //a través de la consola serie del IDE de Arduino
}
 
void loop()
{
  float voltaje, gradosC, gradosF;  //Declaramos estas variables tipo float para guardar los valores de lectura
                                    //del sensor, así como las conversiones a realizar para convertir a grados
                                    //centígrados y a grados Fahrenheit
                                     
  voltaje = analogRead(0) * 0.004882814;  //Con esta operación lo que hacemos es convertir el valor que nos devuelve
                                           //el analogRead(5) que va a estar comprendido entre 0 y 1023 a un valor
                                           //comprendido entre los 0.0 y los 5.0 voltios
                                            
  gradosC = (voltaje - 0.5) * 100.0;  //Gracias a esta fórmula que viene en el datasheet del sensor podemos convertir
                                       //el valor del voltaje a grados centigrados
                                        
  gradosF = ((voltaje - 0.5) * 100.0) * (9.0/5.0) + 32.0;  //Gracias a esta fórmula que viene en el datasheet del sensor podemos convertir
                                                           //el valor del voltaje a grados Fahrenheit
                                                            
  //Mostramos mensaje con valores actuales de humedad y temperatura, asi como maximos y minimos de cada uno de ellos
  Serial.print("Medidas actuales\n");
  Serial.print("C: "); 
  Serial.print(gradosC);
  Serial.print("\tF: "); 
  Serial.print(gradosF);
  Serial.print("\tV: "); 
  Serial.print(voltaje);
  //Comprobacion de maximos y minimos de humedad y temperatura
  if (maxC < gradosC)
    maxC = gradosC;
  if (gradosC < minC)
    minC = gradosC;
  if (maxF < gradosF)
    maxF = gradosF;
  if (gradosF < minF)
    minF = gradosF;
  if (maxV < voltaje)
    maxV = voltaje;
  if (voltaje < minV)
    minV = voltaje;
  Serial.print("\nMedidas maximas\n");
  Serial.print("C: "); 
  Serial.print(maxC);
  Serial.print("\tF: "); 
  Serial.print(maxF);
  Serial.print("\tV: "); 
  Serial.print(maxV);
  Serial.print("\nMedidas minimas\n");
  Serial.print("C: "); 
  Serial.print(minC);
  Serial.print("\tF: "); 
  Serial.print(minF);
  Serial.print("\tV: "); 
  Serial.print(minV);
  Serial.print("\n\n");
  delay(2000);  //Usamos un retardo de 2 segundos entre lectura y lectura  
}
