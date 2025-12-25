#include <AFMotor.h>
#include <Servo.h>

// ==================== КОНФИГУРАЦИЯ ====================

// Конфигурация шаговых двигателей
const int MOTOR_STEPS = 300;        // Количество шагов на оборот (1.8 градуса)
const float WHEEL_CIRCUMFERENCE = 60.0 * 3.14159; // Диаметр колеса в мм * пи

// Создание объектов шаговых двигателей
AF_Stepper leftMotor(MOTOR_STEPS, 1);
AF_Stepper rightMotor(MOTOR_STEPS, 2);

// Конфигурация ультразвукового датчика HC-SR04
const int TRIG_PIN = 3;
const int ECHO_PIN = 2;

// Конфигурация сервопривода для вращения датчика
const int SERVO_PIN = 9;
Servo distanceSensorServo;

// Параметры трубы
const float PIPE_DIAMETER = 200.0;      // Диаметр трубы в мм
const float WALL_THICKNESS = 4.0;       // Толщина стенки трубы в мм
const float EXPECTED_DISTANCE = PIPE_DIAMETER / 2 + WALL_THICKNESS;

// Параметры сервопривода
const int SERVO_START_ANGLE = 10;        // Начальный угол
const int SERVO_END_ANGLE = 160;        // Конечный угол
const int SERVO_DELAY = 1000;            // Задержка между движениями сервопривода (мс)

// Параметры обнаружения дефектов
const float CRACK_THRESHOLD = 5.0;      // Порог обнаружения трещин в мм
const float MAX_DISTANCE = 400.0;       // Максимальная дистанция измерения в мм
const float INSPECTION_DISTANCE = 10000.0; // Дистанция инспекции в мм
const float MEASUREMENT_INTERVAL = 100.0;  // Интервал между измерениями в мм

// Скорость и параметры движения
const int MOTOR_SPEED = 400;            // Скорость в RPM (обороты в минуту)
const int STEP_DELAY = 3;              // Задержка между шагами для плавности (мс)

// ==================== ПЕРЕМЕННЫЕ СОСТОЯНИЯ ====================

float traveledDistance = 0.0;           // Пройденное расстояние
float measurementPoint = 0.0;           // Точка последнего измерения
bool crackDetected = false;             // Флаг обнаружения трещины
int stepsPerMM;                         // Шагов на миллиметр
int currentServoAngle = SERVO_START_ANGLE; // Текущий угол сервопривода
unsigned long lastStepTime = 0;         // Время последнего шага
long totalSteps = 0;                    // Общее количество сделанных шагов

// ==================== НАСТРОЙКА ====================

void setup() {
  Serial.begin(9600);
  Serial.println(F("========================================"));
  Serial.println(F("РОБОТ ДЛЯ ИНСПЕКЦИИ ТРУБ"));
  Serial.println(F("========================================"));
  
  // Расчет шагов на миллиметр
  stepsPerMM = MOTOR_STEPS / WHEEL_CIRCUMFERENCE;
  
  // Настройка скорости двигателей
  leftMotor.setSpeed(MOTOR_SPEED);
  rightMotor.setSpeed(MOTOR_SPEED);
  
  // Настройка ультразвукового датчика
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  
  // Настройка сервопривода
  distanceSensorServo.attach(SERVO_PIN);
  distanceSensorServo.write(SERVO_START_ANGLE);
  currentServoAngle = SERVO_START_ANGLE;
  delay(5000);
  
  // Вывод информации о конфигурации
  Serial.print(F("Диаметр трубы: "));
  Serial.print(PIPE_DIAMETER);
  Serial.println(F(" мм"));
  
  Serial.print(F("Ожидаемое расстояние до стенки: "));
  Serial.print(EXPECTED_DISTANCE);
  Serial.println(F(" мм"));
  
  Serial.print(F("Порог обнаружения трещин: "));
  Serial.print(CRACK_THRESHOLD);
  Serial.println(F(" мм"));
  
  Serial.print(F("Дистанция инспекции: "));
  Serial.print(INSPECTION_DISTANCE / 1000);
  Serial.println(F(" м"));
  
  Serial.print(F("Шагов на мм: "));
  Serial.println(stepsPerMM);
  
}

// ==================== ОСНОВНОЙ ЦИКЛ ====================

void loop() {
  if (traveledDistance < INSPECTION_DISTANCE && !crackDetected) {
    // Фаза движения и инспекции
    moveForward();
    
    // Проверка необходимости выполнения измерения
    if (traveledDistance - measurementPoint >= MEASUREMENT_INTERVAL) {
      stopForMeasurement();
      performDualMeasurement();
      measurementPoint = traveledDistance;
      
      Serial.print(F("Пройдено: "));
      Serial.print(traveledDistance);
      Serial.print(F(" мм ("));
      Serial.print(traveledDistance / 1000);
      Serial.println(F(" м)"));
    }
    
    // Обновление пройденной дистанции
    updateTraveledDistance();
    
  } else if (crackDetected) {
    // Обнаружена трещина - аварийная остановка
    emergencyStop();
    
  } else {
    // Инспекция завершена успешно
    inspectionComplete();
  }
}

// ==================== ФУНКЦИИ ДВИЖЕНИЯ ====================

void moveForward() {
  if (millis() - lastStepTime > STEP_DELAY) {
    leftMotor.step(10, FORWARD, SINGLE);
    rightMotor.step(10, FORWARD, SINGLE);
    lastStepTime = millis();
    totalSteps++;
  }
}

void moveBackward() {
  if (millis() - lastStepTime > STEP_DELAY) {
    leftMotor.step(10, BACKWARD, SINGLE);
    rightMotor.step(10, BACKWARD, SINGLE);
    lastStepTime = millis();
    totalSteps--;
  }
}

void stopMotors() {
  // Устанавливаем скорость 0 для остановки
  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
  delay(100);
  // Возвращаем рабочую скорость
  leftMotor.setSpeed(MOTOR_SPEED);
  rightMotor.setSpeed(MOTOR_SPEED);
}

void stopForMeasurement() {
  // Кратковременная остановка для точного измерения
  stopMotors();
  delay(50);
}

// ==================== ФУНКЦИИ ИЗМЕРЕНИЙ ====================

float measureDistance() {
  // Отправка импульса на trig
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Измерение времени отклика
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Таймаут 30 мс
  
  // Преобразование времени в расстояние (мм)
  float distance = duration * 0.0343 / 2.0;
  
  // Проверка на корректность измерения
  if (distance <= 0 || distance > MAX_DISTANCE) {
    distance = EXPECTED_DISTANCE; // Значение по умолчанию при ошибке
  }
  
  return distance;
}

void checkForCracks(float measuredDistance) {
  float deviation = abs(measuredDistance - EXPECTED_DISTANCE);
  
  if (deviation > CRACK_THRESHOLD) {
    crackDetected = true;
    
    Serial.print(F("ОБНАРУЖЕНА АНОМАЛИЯ! "));
    Serial.print(F("Ожидалось: "));
    Serial.print(EXPECTED_DISTANCE);
    Serial.print(F(" мм, Измерено: "));
    Serial.print(measuredDistance);
    Serial.print(F(" мм, Отклонение: "));
    Serial.print(deviation);
    Serial.print(F(" мм"));
    
    confirmCrackDetection();
  }
}

void confirmCrackDetection() {
  int confirmations = 0;
  const int REQUIRED_CONFIRMATIONS = 3;
  const int CONFIRMATION_ATTEMPTS = 5;
  
  Serial.println(F("Подтверждение обнаружения..."));
  
  for (int i = 0; i < CONFIRMATION_ATTEMPTS; i++) {
    distanceSensorServo.write(SERVO_START_ANGLE);
    delay(SERVO_DELAY);
    currentServoAngle = SERVO_START_ANGLE;
    
    float distance = measureDistance();
    float deviation = abs(distance - EXPECTED_DISTANCE);
    
    Serial.print(F("Подтверждение "));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.print(distance);
    Serial.print(F(" мм, отклонение: "));
    Serial.print(deviation);
    Serial.println(F(" мм"));
    
    if (deviation > CRACK_THRESHOLD) {
      confirmations++;
    }
    delay(100);
  }
  
  if (confirmations >= REQUIRED_CONFIRMATIONS) {
    crackDetected = true;
    Serial.println(F("ТРЕЩИНА ПОДТВЕРЖДЕНА!"));
  } else {
    crackDetected = false;
    Serial.println(F("Ложное срабатывание. Продолжаем инспекцию."));
  }
}

void performDualMeasurement() {
  Serial.println(F("--- Начало измерения ---"));
  
  // Измерение в начальном положении (0°)
  distanceSensorServo.write(SERVO_START_ANGLE);
  delay(SERVO_DELAY);
  currentServoAngle = SERVO_START_ANGLE;
  
  float distance1 = measureDistance();
  Serial.print(F("Измерение 1: "));
  Serial.print(distance1);
  Serial.println(F(" мм"));
  
  checkForCracks(distance1);
  if (crackDetected) return;
  
  // Измерение в противоположном положении (180°)
  distanceSensorServo.write(SERVO_END_ANGLE);
  delay(SERVO_DELAY);
  currentServoAngle = SERVO_END_ANGLE;
  
  float distance2 = measureDistance();
  Serial.print(F("Измерение 2: "));
  Serial.print(distance2);
  Serial.println(F(" мм"));
  
  checkForCracks(distance2);
  if (crackDetected) return;
  
  // Возврат в исходное положение
  distanceSensorServo.write(SERVO_START_ANGLE);
  delay(SERVO_DELAY);
  currentServoAngle = SERVO_START_ANGLE;
  
  // Анализ разницы между измерениями
  float measurementDifference = abs(distance1 - distance2);
  if (measurementDifference > CRACK_THRESHOLD * 2) {
    Serial.print(F("Внимание: большая разница между измерениями: "));
    Serial.print(measurementDifference);
    Serial.println(F(" мм"));
  }
  
  Serial.println(F("--- Измерение завершено ---"));
}

// ==================== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ====================

void updateTraveledDistance() {
  traveledDistance = totalSteps / (float)stepsPerMM;
}

void rotate180() {
  Serial.println(F("Поворот на 180 градусов..."));
  
  const float TRACK_WIDTH = 150.0; // Ширина колеи в мм
  const float ROTATION_CIRCUMFERENCE = 3.14159 * TRACK_WIDTH;
  long rotationSteps = (ROTATION_CIRCUMFERENCE / 2) * stepsPerMM;
  
  // Поворот на месте: левый вперед, правый назад
  for (long i = 0; i < rotationSteps; i++) {
    leftMotor.step(1, FORWARD, SINGLE);
    rightMotor.step(1, BACKWARD, SINGLE);
    delay(STEP_DELAY);
  }
  
  Serial.println(F("Поворот завершен"));
}

void smoothServoSweep(int startAngle, int endAngle, int stepDelay = 15) {
  if (startAngle < endAngle) {
    for (int angle = startAngle; angle <= endAngle; angle++) {
      distanceSensorServo.write(angle);
      currentServoAngle = angle;
      delay(stepDelay);
    }
  } else {
    for (int angle = startAngle; angle >= endAngle; angle--) {
      distanceSensorServo.write(angle);
      currentServoAngle = angle;
      delay(stepDelay);
    }
  }
}

// ==================== ФУНКЦИИ УПРАВЛЕНИЯ СОСТОЯНИЕМ ====================

void emergencyStop() {
  stopMotors();
  Serial.println(F("========================================"));
  Serial.println(F("!!! АВАРИЙНАЯ ОСТАНОВКА !!!"));
  Serial.println(F("ОБНАРУЖЕНА ТРЕЩИНА В ТРУБЕ"));
  Serial.println(F("========================================"));
  Serial.print(F("Координата обнаружения: "));
  Serial.print(traveledDistance);
  Serial.print(F(" мм ("));
  Serial.print(traveledDistance / 1000);
  Serial.println(F(" м)"));
  Serial.println(F("========================================"));
  
  // Мигание как аварийный сигнал
  for (int i = 0; i < 10; i++) {
    Serial.println(F("ВНИМАНИЕ! ТРЕЩИНА!"));
    delay(500);
  }
  
  Serial.println(F("Начинаем возврат к началу..."));
  returnToStart();
}

void inspectionComplete() {
  stopMotors();
  Serial.println(F("========================================"));
  Serial.println(F("ИНСПЕКЦИЯ УСПЕШНО ЗАВЕРШЕНА"));
  Serial.println(F("========================================"));
  Serial.print(F("Проверено: "));
  Serial.print(traveledDistance);
  Serial.print(F(" мм ("));
  Serial.print(traveledDistance / 1000);
  Serial.println(F(" м)"));
  Serial.println(F("Дефектов не обнаружено."));
  Serial.println(F("========================================"));
  
  returnToStart();
}

void returnToStart() {
  Serial.println(F("========================================"));
  Serial.println(F("ВОЗВРАТ В ИСХОДНУЮ ТОЧКУ"));
  Serial.println(F("========================================"));
  
  // Возврат сервопривода в начальное положение
  distanceSensorServo.write(SERVO_START_ANGLE);
  currentServoAngle = SERVO_START_ANGLE;
  delay(500);
  
  // Поворот на 180 градусов
  rotate180();
  delay(1000);
  
  // Возврат по пройденному пути
  Serial.print(F("Дистанция возврата: "));
  Serial.print(traveledDistance);
  Serial.println(F(" мм"));
  
  long returnSteps = totalSteps;
  
  Serial.print(F("Выполняется возврат... "));
  for (long i = 0; i < returnSteps; i++) {
    moveBackward();
    delay(STEP_DELAY);
    
    // Прогресс каждые 10%
    if (i % (returnSteps / 10) == 0 && i != 0) {
      Serial.print((i * 100) / returnSteps);
      Serial.print(F("% "));
    }
  }
  
  stopMotors();
  Serial.println(F("100%"));
  Serial.println(F("========================================"));
  Serial.println(F("ВОЗВРАТ ЗАВЕРШЕН"));
  Serial.println(F("Робот в исходной позиции"));
  Serial.println(F("========================================"));
  
  // Окончательная остановка
  while (true) {
    delay(1000);
  }
}