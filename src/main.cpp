#include <Arduino.h>
#include <Preferences.h>
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"

Preferences preferences;

MPU6050 mpu;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
Quaternion qReference; // Quaternion de referência
VectorFloat gravity;
float ypr[3];

const int buttonPin = 13; // Pino do botão


void setup()
{
    Serial.begin(115200);
    pinMode(buttonPin, INPUT_PULLUP);
    Wire.begin();

    mpu.initialize();
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    devStatus = mpu.dmpInitialize();

    if (devStatus == 0)
    {
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        packetSize = mpu.dmpGetFIFOPacketSize();
        preferences.begin("mpu-calib", false); // Abre o armazenamento com o namespace "mpu-calib"

        if (preferences.getBytesLength("qReference") == sizeof(qReference))
        {
            preferences.getBytes("qReference", &qReference, sizeof(qReference));
            Serial.println("Calibração carregada da memória flash");
        }
        else
        {
            qReference = Quaternion(1, 0, 0, 0);
            Serial.println("Calibração padrão utilizada");
        }
        // qReference = Quaternion(1, 0, 0, 0);

        dmpReady = true;
    }
    else
    {
        Serial.println("DMP Initialization failed (code " + String(devStatus) + ")");
    }

    delay(6000);
}

void loop()
{
    // Verifique se há novos dados do DMP disponíveis
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    {
        VectorFloat gravity; // Vetor de gravidade
        float ypr[3];        // Armazena os valores de yaw, pitch e roll

        Quaternion qCurrent;

        mpu.dmpGetQuaternion(&qCurrent, fifoBuffer);

        if (digitalRead(buttonPin) == LOW)
        {
            qReference = qCurrent;
            preferences.putBytes("qReference", &qReference, sizeof(qReference));

            while (digitalRead(buttonPin) == LOW)
            {
                delay(10);
            }
        }

        // Calcula a orientação relativa
        Quaternion qRelative = qCurrent.getProduct(qReference.getConjugate());

        mpu.dmpGetGravity(&gravity, &qRelative);
        mpu.dmpGetYawPitchRoll(ypr, &qRelative, &gravity);

        float pitch = ypr[1] * 180 / M_PI;
        float roll = ypr[2] * 180 / M_PI;

        // Imprime os valores de inclinação (pitch e roll)
        roll = abs(roll);
        pitch = abs(pitch);

        Serial.print("Pitch: ");
        Serial.print(pitch);
        Serial.print("\tRoll: ");
        Serial.println(roll);
    }
    delay(10);
}
