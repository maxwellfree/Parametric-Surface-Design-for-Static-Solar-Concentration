#include <DFRobot_TFmini.h>       // Librería para manejar el sensor LIDAR TFmini
#include <Wire.h>                 // Librería para comunicación I2C (usada por el LCD)
#include <LCD.h>                  // Librería general para manejar pantallas LCD
#include <LiquidCrystal_I2C.h>    // Librería específica para pantallas LCD con comunicación I2C
#include <SPI.h>                  // Librería para comunicación SPI (usada para la tarjeta SD)
#include <SD.h>                   // Librería para manejar la tarjeta SD
#include <SoftwareSerial.h>       // Librería para emular puertos serie adicionales
#include <math.h>                 // Librería matemática, usada para funciones como sqrt, atan2, etc.
#include <EasyBuzzer.h>           // Librería para manejar el buzzer (emisor de pitidos)
#include <Servo.h>            		// Librería para controlar los servos
#include <MPU6050.h>  // Librería para manejar el GY-521

#define Buzzer_activo 3  // Definir el pin 3 como el que controla el buzzer
#define DEG_TO_RAD 0.017453292519943295769236907684886  // Factor de conversión de grados a radianes

// Definir puerto serial para el sensor TFmini (LIDAR)
SoftwareSerial mySerial(0, 1);    // Configuración de los pines RX y TX para el LIDAR

// Inicialización del display LCD con comunicación I2C
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7); // Configuración de los pines del LCD usando I2C

// Instancia del sensor LIDAR TFmini
DFRobot_TFmini TFmini;

// Variables para el procesamiento de la distancia medida por el sensor
const int numReadings = 5;        // Número de lecturas para promediar la distancia
uint16_t distance, strength;      // Variables para la distancia y fuerza de la señal del LIDAR
uint16_t readings[numReadings];   // Array para almacenar múltiples lecturas de distancia
int readIndex = 0;                // Índice de la lectura actual
uint16_t total = 0;               // Suma de todas las lecturas para calcular el promedio
uint16_t averageDistance = 0;     // Distancia promedio calculada

// Configuración de la tarjeta SD
const int chipSelect = 10;        // Pin de selección para la tarjeta SD
const int maxFilas = 10;          // Número máximo de filas de datos que se pueden almacenar
const int columnas = 3;           // Número de columnas en la matriz de datos (coordenadas x, y, z)
int matriz[maxFilas][columnas];   // Matriz que almacenará los datos de las coordenadas

// Variables para manejar los ángulos de movimiento
float anguloThetaObjetivo = 0;  // Ángulo azimutal objetivo (horizontal, 0° a 360°)
float anguloPhiObjetivo;    // Ángulo polar objetivo (vertical, 0° a 90°)
float anguloThetaObjetivoRad;   // Ángulo azimutal convertido a radianes
float anguloPhiObjetivoRad;     // Ángulo polar convertido a radianes

// Estructura para almacenar coordenadas cartesianas (x, y, z) y la distancia a un punto
struct Punto {
    int x, y, z;        // Coordenadas cartesianas
    float distancia;    // Distancia desde el origen al punto
};

// Variable que almacena la distancia medida cuando se presiona el botón del joystick
float Zref = 0; 

// Matriz que almacena las coordenadas de los 8 puntos más cercanos
Punto puntosCercanos[8];  // Array de estructuras "Punto" que almacenan las coordenadas y distancias

// Definir el pin del botón del joystick
const int botonPin = 2;  // Pin digital conectado al botón del joystick
const int xPin = A0;     // Pin analógico para el eje X del joystick
const int yPin = A1;     // Pin analógico para el eje Y del joystick
int xVal, yVal;          // Variables para almacenar los valores del joystick
int buttonState;         // Estado actual del botón

// Parámetros del joystick
const int rangoMuerto = 20;     // Rango muerto (zona donde no se detectan movimientos) alrededor del centro
const int centroJoystick = 512; // Valor central teórico del joystick (0-1023)

int lastButtonState = HIGH;        // Estado anterior del botón (para debounce)
unsigned long lastDebounceTime = 0;// Tiempo del último cambio de estado del botón
const unsigned long debounceDelay = 50; // Tiempo de espera para debounce

// Velocidad de cambio de los ángulos cuando se mueve el joystick
float velocidadCambioTheta = 1;  // Velocidad de cambio para el ángulo azimutal
float velocidadCambioPhi = 1;    // Velocidad de cambio para el ángulo polar

// Estados posibles del sistema (máquina de estados)
enum Estado {AJUSTE_ANGULOS, CAMBIAR_POSICION, MEDIR_DISTANCIA, LEER_INTERPOLAR};
Estado estadoActual = AJUSTE_ANGULOS;  // Estado inicial del sistema: ajuste de ángulos
int con = 0;  // Contador auxiliar

// Configuración del buzzer
unsigned long tiempoAnterior = 0;  // Tiempo anterior (para manejar el intervalo de pitidos)
const long intervaloPitido = 1;    // Intervalo entre pitidos del buzzer en milisegundos

//Rele laser
const int pinRELE = 9;

Servo servoAzi;
Servo servoPolar;
int pinServoPolar = 7;
int pinServoAzi = 8;
int valorDetener = 0;    // Valor ajustado para detener el servo (ajústalo si es necesario)
int velocidadServo = 180;      // Velocidad aproximada del servo en grados por segundo

float anguloActualPolar = 0;          // Ángulo actual del servo
float anguloActualAzi = 0;          // Ángulo actual del servo

// Variables para el giroscopio GY-521
MPU6050 mpu;
float pitch = 0, roll = 0;  // Ángulos de inclinación


void setup() {

    //El giroscopo se inicializa
    inicializarGiroscopio();  
 
      // Configuración de pines de los servos
    servoAzi.attach(pinServoAzi);
    servoPolar.attach(pinServoPolar);

    // Inicializar los servos en posición neutral
    servoAzi.write(valorDetener);
    servoPolar.write(valorDetener);

    // Inicializar el sensor TFmini (LIDAR) utilizando la comunicación serial por software
    TFmini.begin(mySerial);

    //Inicializar la pantalla LCD
    lcd.setBacklightPin(3, POSITIVE);  // Configura el pin de retroiluminación como positivo
    lcd.setBacklight(HIGH);            // Enciende la retroiluminación del LCD
    lcd.begin(16, 2);                  // Configura el tamaño del LCD (16 columnas y 2 filas)
    lcd.clear();                       // Limpia el contenido del LCD

    // Inicializar el array de lecturas de distancia con ceros (para promediar distancias)
    for (int i = 0; i < numReadings; i++) {
        readings[i] = 0;  // Inicializa cada posición del array en 0
    }

    // Configuración de los pines del joystick como entrada
    pinMode(xPin, INPUT);      // Configura el pin del eje X del joystick como entrada
    pinMode(yPin, INPUT);      // Configura el pin del eje Y del joystick como entrada
    pinMode(botonPin, INPUT_PULLUP);  // Configura el pin del botón del joystick con resistencia interna pull-up

    // Configuración del buzzer
    EasyBuzzer.setPin(Buzzer_activo);  // Asigna el pin del buzzer
    EasyBuzzer.stopBeep();             // Detiene cualquier beep que esté sonando al inicio

    // //Rele laser
    // pinMode(pinRELE,OUTPUT);
    // digitalWrite(pinRELE,HIGH);
    

    // // Bucle while que mide la distancia continuamente hasta que se pulse el botón del joystick
    while (digitalRead(botonPin) == HIGH) {
        // Mientras el botón no se haya presionado, medir la distancia usando el sensor TFmini
        medirDistancia();   // Llama a la función que mide la distancia
        delay(100);         // Pausa de 100 milisegundos para evitar lecturas excesivas
    }
    // digitalWrite(pinRELE,LOW);

    // Una vez que se presiona el botón, limpia la pantalla LCD
    lcd.clear();

    // // Muestra la distancia almacenada en la variable Zref en el LCD
    lcd.setCursor(0, 0);          // Posiciona el cursor en la primera fila, primera columna
    lcd.print("Zref = ");         // Imprime el texto "Zref = " en el LCD
    lcd.setCursor(0, 1);          // Posiciona el cursor en la segunda fila
    lcd.print(averageDistance);   // Muestra la distancia promedio medida
    lcd.print(" cm");             // Añade la unidad de medida "cm" (centímetros)
    
    // Almacena la distancia promedio en la variable Zref para referencia futura
    Zref = averageDistance;

    // Pausa de 2.5 segundos para mostrar la distancia en la pantalla antes de continuar
    delay(2500);
}

void loop() {


    // Leer el estado actual del botón (con debounce para evitar falsos cambios)
    int reading = digitalRead(botonPin);

    // Si el botón ha sido presionado y se ha hecho al menos una lectura previa
    if (reading == LOW && con > 1) {
         // Cambia de estado cuando el botón es presionado (mediante una máquina de estados)
         cambiarEstado();  // Cambia entre los diferentes estados del sistema
    }

    // Ejecutar la lógica según el estado actual del sistema
    switch (estadoActual) {
        case AJUSTE_ANGULOS:
            // Si el estado actual es de ajuste de ángulos, ajusta los ángulos según el joystick             
            ajustarAngulos();                        
            break;
        case CAMBIAR_POSICION:
            // Me voy a la posicion ANGULAR
            moverServoPolar(anguloPhiObjetivo);
            moverServoAzi(anguloThetaObjetivo);
            // Después de leer e interpolar, cambia al siguiente estado
            delay(1000);
            cambiarEstado();
            break;        
        case MEDIR_DISTANCIA:
            // Si el estado actual es medir la distancia, mide la distancia usando el LIDAR
             medirDistancia();
            break;
        case LEER_INTERPOLAR:
            // Si el estado actual es leer e interpolar, realiza la lectura e interpolación de puntos            
             leerEInterpolar();
            // Después de leer e interpolar, cambia al siguiente estado
            cambiarEstado();
    }

    //Incrementar el contador 'con' en cada ciclo hasta llegar a 10
    //Este contador puede usarse como control para otras funciones
    if (con < 10) {
        con += 1;  // Aumenta el contador solo si es menor que 10
    }
}

// Función para leer y comparar puntos desde la tarjeta SD con corrección de inclinación
void leerYCompararPuntosSD(int chipSelect, int matriz[][3], int maxFilas, float anguloThetaObjetivo, float anguloPhiObjetivo, Punto puntosCercanos[8]) {
    const float angulo90 = 90 * DEG_TO_RAD;      // 90 grados en radianes
    const float angulo180 = 180 * DEG_TO_RAD;    // 180 grados en radianes
    const float angulo270 = 270 * DEG_TO_RAD;    // 270 grados en radianes
    float theta = 0;                             // Ángulo azimutal
    float distanciasAngulares[8];                // Array para las distancias angulares

    // Inicializar las distancias angulares con un valor alto
    for (int i = 0; i < 8; i++) {
        distanciasAngulares[i] = 99999.0;
    }

    // Inicializar tarjeta SD
    if (!SD.begin(chipSelect)) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("SD Error");
        while (true);  // Bloquear ejecución en caso de error
    }
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SD OK");

    // Abrir archivo con coordenadas
    File archivo = SD.open("coor.dat");
    if (!archivo) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("File Error");
        while (true);  // Bloquear ejecución en caso de error
    }

    // Leer inclinación del giroscopio
    leerGiroscopio();  // Obtiene los valores de `pitch` y `roll`

    // Procesar las coordenadas del archivo
    while (archivo.available()) {
        int filas = 0;

        while (archivo.available() && filas < maxFilas) {
            String linea = archivo.readStringUntil('\n');
            int x = linea.substring(0, linea.indexOf(',')).toInt();
            int y = linea.substring(linea.indexOf(',') + 1, linea.lastIndexOf(',')).toInt();
            int z = linea.substring(linea.lastIndexOf(',') + 1).toInt();

            // Restar Zref para corregir la nube de puntos
            z -= Zref;

            // Corregir las coordenadas usando el giroscopio
            float x_corr = x * cos(pitch * DEG_TO_RAD) + z * sin(pitch * DEG_TO_RAD);
            float y_corr = y * cos(roll * DEG_TO_RAD) - z * sin(roll * DEG_TO_RAD);
            float z_corr = z * cos(pitch * DEG_TO_RAD) - y * sin(roll * DEG_TO_RAD);

            // Calcular radio (r) y ángulos theta y phi
            float r = sqrt(x_corr * x_corr + y_corr * y_corr + z_corr * z_corr);

            // Calcular ángulo azimutal (theta)
            if (x_corr >= 0 && y_corr >= 0) {
                theta = atan2(abs(x_corr), abs(y_corr));
            } else if (x_corr < 0 && y_corr >= 0) {
                theta = angulo90 + atan2(abs(x_corr), abs(y_corr));
            } else if (x_corr < 0 && y_corr < 0) {
                theta = angulo180 + atan2(abs(x_corr), abs(y_corr));
            } else {
                theta = angulo270 + atan2(abs(x_corr), abs(y_corr));
            }

            // Calcular ángulo polar (phi)
            float phi = (M_PI / 2) - acos(z_corr / r);

            // Calcular distancia angular
            float distanciaAngular = sqrt(pow(theta - anguloThetaObjetivo, 2) + pow(phi - anguloPhiObjetivo, 2));

            // Actualizar puntos cercanos
            for (int i = 0; i < 8; i++) {
                if (distanciaAngular < distanciasAngulares[i]) {
                    for (int j = 7; j > i; j--) {
                        distanciasAngulares[j] = distanciasAngulares[j - 1];
                        puntosCercanos[j] = puntosCercanos[j - 1];
                    }
                    distanciasAngulares[i] = distanciaAngular;
                    puntosCercanos[i].x = round(x_corr);
                    puntosCercanos[i].y = round(y_corr);
                    puntosCercanos[i].z = round(z_corr);
                    puntosCercanos[i].distancia = distanciaAngular;
                    break;
                }
            }
            filas++;
        }
    }
    archivo.close();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("File Closed");
}

// Función para interpolar un punto utilizando los 8 puntos más cercanos
Punto interpolarPunto(Punto puntosCercanos[8]) {
    Punto puntoInterpolado;  // Estructura para almacenar el punto interpolado resultante
    float sumaPesos = 0;     // Suma total de los pesos inversos a la distancia
    float sumaPesosX = 0;    // Suma ponderada de las coordenadas X
    float sumaPesosY = 0;    // Suma ponderada de las coordenadas Y
    float sumaPesosZ = 0;    // Suma ponderada de las coordenadas Z
    float exponente = 2.0;   // Exponente para ponderación inversa a la distancia

    // Calcular los pesos inversos a la distancia para cada uno de los 8 puntos
    for (int i = 0; i < 8; i++) {
        if (puntosCercanos[i].distancia == 0) {
            // Si la distancia es 0, significa que el punto coincide exactamente, por lo tanto, retornamos ese punto
            return puntosCercanos[i];
        }

        // Calcular el peso como el inverso de la distancia elevada al exponente
        float peso = 1.0 / pow(puntosCercanos[i].distancia, exponente);
        sumaPesos += peso;  // Sumar el peso
        sumaPesosX += peso * puntosCercanos[i].x;  // Sumar el peso ponderado para X
        sumaPesosY += peso * puntosCercanos[i].y;  // Sumar el peso ponderado para Y
        sumaPesosZ += peso * puntosCercanos[i].z;  // Sumar el peso ponderado para Z
    }

    // Calcular las coordenadas interpoladas dividiendo las sumas ponderadas por la suma de los pesos
    puntoInterpolado.x = round(sumaPesosX / sumaPesos);  // Redondear el valor interpolado de X
    puntoInterpolado.y = round(sumaPesosY / sumaPesos);  // Redondear el valor interpolado de Y
    puntoInterpolado.z = round(sumaPesosZ / sumaPesos);  // Redondear el valor interpolado de Z

    return puntoInterpolado;  // Devolver el punto interpolado
}

// Función para cambiar entre los diferentes estados del sistema
void cambiarEstado() {
    // Cambiar entre los diferentes estados en orden secuencial
    if (estadoActual == AJUSTE_ANGULOS) {
        estadoActual = CAMBIAR_POSICION;  // Si estamos en el estado de ajuste de ángulos, pasamos al estado de medir la distancia
    } else if (estadoActual == CAMBIAR_POSICION) {        
        estadoActual = MEDIR_DISTANCIA;
    } else if (estadoActual == MEDIR_DISTANCIA) {
        estadoActual = LEER_INTERPOLAR;  // Si estamos midiendo la distancia, pasamos al estado de leer e interpolar puntos
    } else if (estadoActual == LEER_INTERPOLAR) {
        estadoActual = AJUSTE_ANGULOS;   // Si estamos leyendo e interpolando, volvemos al estado de ajuste de ángulos
    }
}

void ajustarAngulos() {
    // Leer los valores del joystick (X e Y)
    xVal = analogRead(xPin);  // Leer el valor del eje X del joystick (rango 0-1023)
    yVal = analogRead(yPin);  // Leer el valor del eje Y del joystick (rango 0-1023)

    // Ajustar el ángulo azimutal (anguloThetaObjetivo) según el valor del joystick en el eje Y
    if (abs(yVal - centroJoystick) > rangoMuerto) {
        anguloThetaObjetivo += velocidadCambioTheta * ((yVal - centroJoystick) / 512.0);
        anguloThetaObjetivo = constrain(anguloThetaObjetivo, 0, 180);  // Limitar el rango del ángulo
    }

    // Ajustar el ángulo polar (anguloPhiObjetivo) según el valor del joystick en el eje X
    if (abs(xVal - centroJoystick) > rangoMuerto) {
        anguloPhiObjetivo += velocidadCambioPhi * ((xVal - centroJoystick) / 512.0);
        anguloPhiObjetivo = constrain(anguloPhiObjetivo, 0, 180);  // Limitar el rango del ángulo
    }

    // Mover los servos a los nuevos ángulos calculados
    moverServoAzi(anguloThetaObjetivo);
    moverServoPolar(anguloPhiObjetivo);

    // Mostrar los ángulos ajustados en el LCD
    lcd.setCursor(0, 0);
    lcd.print("Azi:"); lcd.print(anguloThetaObjetivo); lcd.print(" Gra");
    lcd.setCursor(0, 1);
    lcd.print("Pol:"); lcd.print(anguloPhiObjetivo); lcd.print(" Gra");

    delay(100);  // Esperar un poco antes de la siguiente lectura
}

// Función para medir la distancia utilizando el sensor LIDAR TFmini
void medirDistancia() {
   
    // Verificar si el sensor LIDAR ha realizado una medición correcta
    if (TFmini.measure()) {
        distance = TFmini.getDistance();   // Obtener la distancia medida por el sensor LIDAR
        strength = TFmini.getStrength();   // Obtener la intensidad de la señal (opcional, se puede usar para verificar la calidad de la medición)

        // Calcular el promedio de las lecturas de distancia para obtener una medición más estable
        total -= readings[readIndex];      // Restar la lectura más antigua de la suma total
        readings[readIndex] = distance;    // Almacenar la nueva lectura de distancia en el array
        total += readings[readIndex];      // Sumar la nueva lectura a la suma total
        readIndex = (readIndex + 1) % numReadings;  // Avanzar al siguiente índice, reiniciando cuando se alcanza el límite
        averageDistance = total / numReadings;  // Calcular la distancia promedio

        // Mostrar la distancia promedio medida en el LCD
        lcd.setCursor(0, 0);               // Establecer el cursor en la primera fila del LCD
        lcd.print("Distancia = ");         // Mostrar "Distancia = " en el LCD
        lcd.setCursor(0, 1);               // Establecer el cursor en la segunda fila del LCD
        lcd.print(averageDistance);        // Mostrar la distancia promedio
        lcd.print(" cm");                  // Añadir la unidad de medida "cm" (centímetros)
        delay(100);                        // Esperar 100 milisegundos antes de la siguiente lectura
        lcd.clear();                       // Limpiar el LCD para la siguiente actualización
    }
}

// Función para leer puntos desde la tarjeta SD y realizar la interpolación
void leerEInterpolar() {

    // Apagamos laser mientras interpolamos
    digitalWrite(pinRELE,LOW);

    //Llamar a la función para leer los puntos desde la tarjeta SD y comparar con los puntos cercanos
    //Se pasa el ángulo azimutal y polar en radianes para encontrar los 8 puntos más cercanos
    leerYCompararPuntosSD(chipSelect, matriz, maxFilas, anguloThetaObjetivoRad, anguloPhiObjetivoRad, puntosCercanos);
    
    // Interpolar un nuevo punto basándonos en los 8 puntos más cercanos obtenidos de la SD
    Punto puntoInterpolado = interpolarPunto(puntosCercanos);  // Obtener el punto interpolado

    // Calcular la diferencia entre la distancia medida por el sensor LIDAR y el punto interpolado
    lcd.clear();  // Limpiar la pantalla LCD
    lcd.setCursor(0, 0);  // Posicionar el cursor en la primera fila
    lcd.print("Delta = ");  // Mostrar el texto "Delta = " en el LCD
    lcd.setCursor(0, 1);  // Posicionar el cursor en la segunda fila

    // Calcular la diferencia en la distancia entre la medida real y la distancia interpolada
    lcd.print(averageDistance - sqrt(puntoInterpolado.x * puntoInterpolado.x +
                                     puntoInterpolado.y * puntoInterpolado.y +
                                     puntoInterpolado.z * puntoInterpolado.z));
    lcd.print(" cm");  // Mostrar la diferencia de distancia en centímetros en el LCD

    // Calcular la diferencia absoluta entre la distancia medida y la interpolada
    float distance = abs(averageDistance - sqrt(puntoInterpolado.x * puntoInterpolado.x +
                                                puntoInterpolado.y * puntoInterpolado.y +
                                                puntoInterpolado.z * puntoInterpolado.z));

    // Si la diferencia es menor o igual a 5 cm, activar un pitido con el buzzer
    if (distance <= 5) {
        EasyBuzzer.beep(
            2000,  // Frecuencia del beep en Hz (2000 Hz)
            100,   // Duración del beep (100 ms)
            100,   // Duración del silencio entre beeps (100 ms)
            2,     // Número de beeps por ciclo
            300,   // Duración de la pausa entre ciclos de beep
            1      // Número de ciclos de beep
        );
        EasyBuzzer.update();  // Actualizar el buzzer para que emita el beep
    }
    
    // Pausar 5 segundos (5000 milisegundos) antes de la siguiente operación
    delay(5000);
    lcd.clear();  // Limpiar la pantalla LCD después de la pausa
    EasyBuzzer.stopBeep();  // Detener el beep del buzzer si está activo
}

void moverServoAzi(float anguloThetaObjetivo) {
    // Limitar el ángulo al rango permitido del servo (0 a 180 grados)
    anguloThetaObjetivo = constrain(anguloThetaObjetivo, 0, 180);

    // Mover el servo directamente al ángulo deseado
    servoAzi.write(anguloThetaObjetivo);

    // Actualizar el ángulo actual
    anguloActualAzi = anguloThetaObjetivo;
}

void moverServoPolar(float anguloPhiObjetivo) {
    // Limitar el ángulo al rango permitido del servo (0 a 180 grados)
    anguloPhiObjetivo = constrain(anguloPhiObjetivo, 0, 180);

    // Mover el servo directamente al ángulo deseado
    servoPolar.write(anguloPhiObjetivo);

    // Actualizar el ángulo actual
    anguloActualPolar = anguloPhiObjetivo;
}

// Nueva función: Inicializar el giroscopio
void inicializarGiroscopio() {
    Wire.begin();
    mpu.initialize();

    if (!mpu.testConnection()) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("GYRO ERROR");
        while (true);  // Detener ejecución si no se detecta el giroscopio
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("GYRO OK");
    delay(2000);  // Mostrar el mensaje por 2 segundos
}

// Nueva función: Leer datos del giroscopio y calcular ángulos
void leerGiroscopio() {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);

    // Calcular los ángulos pitch y roll
    pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180 / M_PI;
    roll = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / M_PI;

    // Mostrar los ángulos en el LCD (opcional)
    lcd.setCursor(0, 0);
    lcd.print("Pitch: ");
    lcd.print(pitch);
    lcd.setCursor(0, 1);
    lcd.print("Roll: ");
    lcd.print(roll);
    delay(500);
}

