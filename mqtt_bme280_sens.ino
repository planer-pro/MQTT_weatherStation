#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define BUFFER_SIZE 100
#define BME280_ADDRESS (0x76)
#define SEALEVELPRESSURE_HPA (1013.25) //����������� �����������
#define LED 2

Adafruit_BME280 bme; // I2C

const char *ssid = "ssid";  // ��� ������ ����� �������
const char *pass = "pass"; // ������ �� ����� �������

const char *mqtt_server = "serv"; // ��� ������� MQTT
const int mqtt_port = 12345; // ���� ��� ����������� � ������� MQTT
const char *mqtt_user = "user"; // ���� �� ������
const char *mqtt_pass = "pass"; // ������ �� �������

float temp = 0.0;
float hum = 0.0;
float press = 0.0;
float alt = 0.0;

unsigned long prefMillis = 0;

WiFiClient wclient;
PubSubClient client(wclient, mqtt_server, mqtt_port);

void callback(const MQTT::Publish& pub)
{
	/*Serial.print(pub.topic());   // ������� � ������ ���� �������� ������
	Serial.print(" => ");
	Serial.print(pub.payload_string()); // ������� � ������ ���� �������� ���������� ������

	String payload = pub.payload_string();

	if (String(pub.topic()) == "test/led") // ��������� �� ������� �� ��� ������ ������ ������
	{
	int stled = payload.toInt(); // ����������� ���������� ������ � ��� integer
	digitalWrite(5, stled);  //  �������� ��� ��������� ��������� � ���������� �� ���������� �������� ������
	}*/
}

void BmeDataSend()
{
	if (millis() - prefMillis >= 2000)
	{
		digitalWrite(LED, LOW);
		temp = bme.readTemperature() - 7.0;   // �������� �������� �����������. 2.3 ������������� (-7 � �������)
		hum = bme.readHumidity();   // �������� �������� ���������
		press = bme.readPressure() / 100.0F;   // �������� �������� ��������
		alt = bme.readAltitude(SEALEVELPRESSURE_HPA); // �������� �����. ������

		Serial.println(temp);
		Serial.println(hum);
		Serial.println(press);
		Serial.println(alt);

		client.publish("outdoor/sensors/bme280_temp", String(temp)); // ���������� � ����� �����������
		client.publish("outdoor/sensors/bme280_hum", String(hum)); // ���������� � ����� ��������� %
		client.publish("outdoor/sensors/bme280_press", String(press)); // ���������� � ����� �������� ���
		client.publish("outdoor/sensors/bme280_alt", String(alt)); // ���������� � ����� ������

		prefMillis = millis();
	}
	digitalWrite(LED, HIGH);
}

void setup()
{
	Serial.begin(115200);
	Serial.println(F("BME280 test"));
	pinMode(LED, OUTPUT);

	if (!bme.begin(BME280_ADDRESS)) 
	{
		Serial.println("Could not find a valid BME280 sensor, check wiring!");
		while (1);
	}
}

void loop()
{
	// ������������ � wi-fi
	if (WiFi.status() != WL_CONNECTED)
	{
		Serial.print("Connecting to ");
		Serial.print(ssid);
		Serial.println("...");
		WiFi.begin(ssid, pass);

		if (WiFi.waitForConnectResult() != WL_CONNECTED)
			return;
		Serial.println("WiFi connected");
	}

	// ������������ � MQTT �������
	if (WiFi.status() == WL_CONNECTED)
	{
		if (!client.connected())
		{
			Serial.println("Connecting to MQTT server");
			if (client.connect(MQTT::Connect("arduinoClient2").set_auth(mqtt_user, mqtt_pass)))
			{
				Serial.println("Connected to MQTT server");
				//client.set_callback(callback);
				//client.subscribe("test/led"); // �������������� �� ����� � ������� ��� ����������
			}
			else
			{
				Serial.println("Could not connect to MQTT server");
			}
		}

		if (client.connected())
		{
			client.loop();
			BmeDataSend();
		}
	}
}
