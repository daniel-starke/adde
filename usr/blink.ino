#define PIN_LED 9

static bool state = false;

struct StaticTest {
	StaticTest() {
		Serial.print("millis() = ");
		Serial.println(millis());
		
		pinMode(PIN_LED, OUTPUT);
		
		for (uint8_t i = 0; i < 10; i++) {
			state = !state;
			digitalWrite(PIN_LED, state ? HIGH : LOW);
			delay(250);
		}
	}
} staticTest;

void setup() {
	pinMode(PIN_LED, OUTPUT);
}

void loop() {
	state = !state;
	digitalWrite(PIN_LED, state ? HIGH : LOW);
	delay(1000);
}
