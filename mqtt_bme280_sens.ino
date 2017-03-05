#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define BUFFER_SIZE 100
#define BME280_ADDRESS (0x76)
#define SEALEVELPRESSURE_HPA (1013.25) //атмосферное стандартное
#define LED 2

Adafruit_BME280 bme; // I2C

const char *ssid = "ssid";  // Имя вайфай точки доступа
const char *pass = "pass"; // Пароль от точки доступа

const char *mqtt_server = "serv"; // Имя сервера MQTT
const int mqtt_port = 12345; // Порт для подключения к серверу MQTT
const char *mqtt_user = "user"; // Логи от сервер
const char *mqtt_pass = "pass"; // Пароль от сервера

float temp = 0.0;
float hum = 0.0;
float press = 0.0;
float alt = 0.0;

unsigned long prefMillis = 0;

WiFiClient wclient;
PubSubClient client(wclient, mqtt_server, mqtt_port);

void callback(const MQTT::Publish& pub)
{
	/*Serial.print(pub.topic());   // выводим в сериал порт название топика
	Serial.print(" => ");
	Serial.print(pub.payload_string()); // выводим в сериал порт значение полученных данных

	String payload = pub.payload_string();

	if (String(pub.topic()) == "test/led") // проверяем из нужного ли нам топика пришли данные
	{
	int stled = payload.toInt(); // преобразуем полученные данные в тип integer
	digitalWrite(5, stled);  //  включаем или выключаем светодиод в зависимоти от полученных значений данных
	}*/
}

void BmeDataSend()
{
	if (millis() - prefMillis >= 2000)
	{
		digitalWrite(LED, LOW);
		temp = bme.readTemperature() - 7.0;   // получаем значение температуры. 2.3 калибровочная (-7 в корпусе)
		hum = bme.readHumidity();   // получаем значение влажности
		press = bme.readPressure() / 100.0F;   // получаем значение давления
		alt = bme.readAltitude(SEALEVELPRESSURE_HPA); // получаем текущ. высоту

		Serial.println(temp);
		Serial.println(hum);
		Serial.println(press);
		Serial.println(alt);

		client.publish("outdoor/sensors/bme280_temp", String(temp)); // отправляем в топик температуру
		client.publish("outdoor/sensors/bme280_hum", String(hum)); // отправляем в топик влажность %
		client.publish("outdoor/sensors/bme280_press", String(press)); // отправляем в топик давление гПа
		client.publish("outdoor/sensors/bme280_alt", String(alt)); // отправляем в топик высоту

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
	// подключаемся к wi-fi
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

	// подключаемся к MQTT серверу
	if (WiFi.status() == WL_CONNECTED)
	{
		if (!client.connected())
		{
			Serial.println("Connecting to MQTT server");
			if (client.connect(MQTT::Connect("arduinoClient2").set_auth(mqtt_user, mqtt_pass)))
			{
				Serial.println("Connected to MQTT server");
				//client.set_callback(callback);
				//client.subscribe("test/led"); // подписывааемся на топик с данными для светодиода
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
