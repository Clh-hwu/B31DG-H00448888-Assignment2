#include <Arduino.h>

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

// Task handles
TaskHandle_t task1Handle, task2Handle, task3Handle, task4Handle, task5Handle, task7Handle, periodicTaskHandle;

// Define global structure to store frequency measurements
struct FrequencyMeasurements {
    long task2Frequency;
    long task3Frequency;
};
FrequencyMeasurements freqMeasurements;

// Define semaphore to protect access to freqMeasurements structure
SemaphoreHandle_t freqMeasurementsSemaphore = NULL;

// Define queues for task communication
QueueHandle_t buttonQueue;

// Constants
const int LED_PIN = 9;                    // LED pin
const int LED_PIN_Analogue_Error = 5;     // LED pin
const int LED_BUTTON = 6;                 // LED pin
const int BUTTON_PIN = 4;                 // Push button pin
const int ANALOG_PIN = 1;                 // Analog input pin
const int Square_Signal_1 = 2;            // Analog input pin
const int Square_Signal_2 = 3;            // Analog input pin
const int LED_TOGGLE_DELAY = 2000;        // LED toggle delay in milliseconds

// Function prototypes
void task1Function(void *pvParameters);
void task2Function(void *pvParameters);
void task3Function(void *pvParameters);
void task4Function(void *pvParameters);
void task5Function(void *pvParameters);
void task7Function(void *pvParameters);
void periodicTask(void *pvParameters);
void cpuWork(unsigned long pvParameters);
void debounceButton();

int buttonState = HIGH; // Declaring buttonState as a global variable

void setup() {
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);
    pinMode(LED_PIN_Analogue_Error, OUTPUT);
    pinMode(LED_BUTTON, OUTPUT);
    pinMode(BUTTON_PIN, INPUT);
    pinMode(ANALOG_PIN, INPUT);
    pinMode(Square_Signal_1, INPUT);
    pinMode(Square_Signal_2, INPUT);

    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), debounceButton, FALLING);

    // Initialize semaphore
    freqMeasurementsSemaphore = xSemaphoreCreateMutex();
    if (freqMeasurementsSemaphore == NULL) {
        Serial.println("Semaphore creation failed!");
        while (1); // halt if semaphore creation failed
    }

    // Create queues
    buttonQueue = xQueueCreate(10, sizeof(freqMeasurements));
    if (buttonQueue == NULL) {
        Serial.println("Button queue creation failed!");
        while (1); // halt if queue creation failed
    }

    // Create tasks
    xTaskCreatePinnedToCore(task1Function, "Task1", 2048, NULL, 3, &task1Handle, ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(task2Function, "Task2", 2048, NULL, 4, &task2Handle, ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(task3Function, "Task3", 2048, NULL, 4, &task3Handle, ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(task4Function, "Task4", 2048, NULL, 3, &task4Handle, ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(task5Function, "Task5", 2048, NULL, 2, &task5Handle, ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(task7Function, "Task7", 2048, NULL, 5, &task7Handle, ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(periodicTask, "periodicTask", 2048, NULL, 1, &periodicTaskHandle, ARDUINO_RUNNING_CORE);
}

void loop() {
    // Nothing to do here as everything is handled by tasks
}

// Task functions
void task1Function(void *pvParameters) {
    while (1) {
        digitalWrite(LED_PIN, HIGH);
        delayMicroseconds(180);
        digitalWrite(LED_PIN, LOW);
        delayMicroseconds(40);
        digitalWrite(LED_PIN, HIGH);
        delayMicroseconds(530);
        digitalWrite(LED_PIN, LOW);
        delayMicroseconds(3250);
        vTaskDelay(pdMS_TO_TICKS(4));  // Repeat every 4ms
    }
}

void task2Function(void *pvParameters) {
    const int SQUARE_WAVE_PIN = Square_Signal_1;

    while (1) {
        unsigned long startTime = millis();
        int pulseCount = 0;

        while (millis() - startTime < 20.0) {
            if (digitalRead(SQUARE_WAVE_PIN) == HIGH) {
                pulseCount++;
                while (digitalRead(SQUARE_WAVE_PIN) == HIGH) {}
            }
        }

        long frequency = pulseCount * 1000 / 20.0;

        xSemaphoreTake(freqMeasurementsSemaphore, portMAX_DELAY);
        freqMeasurements.task2Frequency = frequency;
        xSemaphoreGive(freqMeasurementsSemaphore);

        vTaskDelay(pdMS_TO_TICKS(20));  // Repeat every 20ms
    }
}

void task3Function(void *pvParameters) {
    const int SQUARE_WAVE_PIN = Square_Signal_2;

    while (1) {
        unsigned long startTime = millis();
        int pulseCount = 0;

        while (millis() - startTime < 8.0) {
            if (digitalRead(SQUARE_WAVE_PIN) == HIGH) {
                pulseCount++;
                while (digitalRead(SQUARE_WAVE_PIN) == HIGH) {}
            }
        }

        long frequency = pulseCount * 1000 / 8.0;

        xSemaphoreTake(freqMeasurementsSemaphore, portMAX_DELAY);
        freqMeasurements.task3Frequency = frequency;
        xSemaphoreGive(freqMeasurementsSemaphore);

        vTaskDelay(pdMS_TO_TICKS(8));  // Repeat every 8ms
    }
}

void task4Function(void *pvParameters) {
    const int numReadings = 10;
    int readings[numReadings];
    int index = 0;
    long total = 0;

    while (1) {
        int analogValue = analogRead(ANALOG_PIN);
        total -= readings[index];
        readings[index] = analogValue;
        total += readings[index];
        index = (index + 1) % numReadings;
        int average = total / numReadings;

        if (average > 400) {
            digitalWrite(LED_PIN_Analogue_Error, HIGH);
        } else {
            digitalWrite(LED_PIN_Analogue_Error, LOW);
        }
        vTaskDelay(pdMS_TO_TICKS(20));  // Repeat every 20ms
    }
}

void task5Function(void *pvParameters) {
    while (1) {
        xSemaphoreTake(freqMeasurementsSemaphore, portMAX_DELAY);
        long freq2 = map(freqMeasurements.task2Frequency, 0, 3000, 333, 1000);
        long freq3 = map(freqMeasurements.task3Frequency, 0, 3000, 500, 1000);
        freq2 = map(freqMeasurements.task2Frequency, 333, 1000, 0, 99);
        freq3 = map(freqMeasurements.task3Frequency, 500, 1000, 0, 99);
        Serial.print(freq2);
        Serial.printf(",");
        Serial.println(freq3);
        vTaskDelay(pdMS_TO_TICKS(200));  // Repeat every 200ms
        xSemaphoreGive(freqMeasurementsSemaphore);
    }
}

void task7Function(void *pvParameters) {
    while (1) {
        if (xQueueReceive(buttonQueue, &buttonState, portMAX_DELAY) == pdTRUE) {
            digitalWrite(LED_BUTTON, !digitalRead(LED_BUTTON));
              }
        }
}

void periodicTask(void *pvParameters) {
  cpuWork(2); // Perform CPU work for approximately 2 milliseconds
  vTaskDelay(pdMS_TO_TICKS(200));  // Repeat every 20ms
}

// Functions
void cpuWork(int pvParameters) {
    unsigned long t = pvParameters;
    unsigned long endTime = millis() + t;
    // Simulate CPU work for approximately 'time' milliseconds using a for loop
    for (unsigned long start = millis(); millis() - start < t;) {
      // Do nothing, just consume CPU cycles
    }
}

void debounceButton() {
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();
    if (interrupt_time - last_interrupt_time > 50) {
        buttonState = digitalRead(BUTTON_PIN);
        xQueueSend(buttonQueue, &buttonState, 0);
    }
    last_interrupt_time = interrupt_time;
}
