#include <Arduino.h>
#include <GyverOLED.h>
// GyverOLED<SSD1306_128x32, OLED_BUFFER> oled;
// GyverOLED<SSD1306_128x32, OLED_NO_BUFFER> oled;
// GyverOLED<SSD1306_128x64, OLED_BUFFER> oled;
// GyverOLED<SSD1306_128x64, OLED_NO_BUFFER> oled;
// GyverOLED<SSD1306_128x64, OLED_BUFFER, OLED_SPI, 8, 7, 6> oled;
//#include <GyverMAX6675.h>
#include <EncButton.h>
#include <GyverPID.h>
#include <PIDtuner2.h>
#include <GyverWDT.h>
#include <microDS18B20.h>
#include <RunningMedian.h>
#include <EEPROM.h>

// float gettemp();

#define EEPROM_ADDRESS 0

#define DS18B20_PIN 12
#define PWM_PIN 10 // PWMOUT
#define POLLING_PERIOD_MS 500 //period for polling timer, depends on ds18b20 resolution, 750 -- 100 for 12bit -- 9bit resolution
// Пины энкодера
#define A_PIN 2
#define B_PIN 3
#define BTN_PIN 8
// Пины модуля MAX6675K
// #define CLK_PIN 7  // Пин SCK
// #define DATA_PIN 6 // Пин SO
// #define CS_PIN 5   // Пин CS

#define LED_PIN LED_BUILTIN // Пин светодиода для мигания, по умолчанию встроенный

// #define SMOOTH 30 // temp smoothing

// #define B 3950              // B-коэффициент
// #define SERIAL_R 91000      // сопротивление последовательного резистора, 102 кОм
// #define THERMISTOR_R 100000 // номинальное сопротивления термистора, 100 кОм
// #define NOMINAL_T 25        // номинальная температура (при которой TR = 100 кОм)
// const byte tempPin = A7;

MicroDS18B20<DS18B20_PIN> ds18b20;
GyverOLED<SSD1306_128x64, OLED_NO_BUFFER> oled;
//GyverMAX6675<CLK_PIN, DATA_PIN, CS_PIN> sens;
// EncButton2<EB_ENCBTN> enc(INPUT, A_PIN, B_PIN, BTN_PIN);
EncButton<EB_TICK, A_PIN, B_PIN, BTN_PIN> enc;
GyverPID regulator(5, 0.05, 0.01, POLLING_PERIOD_MS);
RunningMedian temp_array = RunningMedian(10);
PIDtuner2 tuner;
uint8_t menu = 0; // menu number (0 - default, 1 - PID Set, 2 - PID Cal)
int16_t tTemp = 25;   // C
// float value = 30;
// float signal = 0;
float t;
uint32_t tmr = 0;
uint8_t calACK;

void setup()
{
  Serial.begin(115200);
  Serial.println("Serial init OK");
  Watchdog.enable(RESET_MODE, WDT_TIMEOUT_4S);
  Serial.println("Watchdog init OK");
  // pinMode(tempPin, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  TCCR1B = TCCR1B & B11111000 | B00000101;
  Serial.println("PWM init OK");
  oled.init();
  oled.setScale(1); // инициализация
  oled.flipV(0);
  oled.flipH(0);
  oled.autoPrintln(true);
  oled.clear();
  Serial.println("oled init OK");
  ds18b20.setResolution(10);
  tuner.setParameters(NORMAL, 25, 30, 20000, 0.5, 1000);
  // Serial.setTimeout(50);
  regulator.setDirection(NORMAL); // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
  regulator.setLimits(0, 1023);    // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255

  EEPROM.get(EEPROM_ADDRESS, tTemp);

  if (tTemp < 0 || tTemp > 100) {
    tTemp = 25;
  }

  regulator.setpoint = tTemp;

  Serial.println("PID regulator init OK");
  enc.setEncType(EB_HALFSTEP);
  Serial.println("Encoder init OK");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  // delay(500);
}

bool tempproc()
{
  //   float tsum = 0;
  //   for (int i = 0; i<SMOOTH;i++){

  //       tsum += gettemp();

  //     }

  //  t = tsum/SMOOTH;
  // if (!sens.readTemp())
  // {
  //   return false;
  // }
  // return true;
  if (!ds18b20.readTemp())
  {
    return false;
  }
  //t = ds18b20.getTemp();
  temp_array.add(ds18b20.getTemp());
  t = temp_array.getMedian();
  Serial.println("$"+(String)((int)t*10)+";");
  ds18b20.requestTemp();
  return true;
}

// float gettemp()
// {
//   int t = analogRead( tempPin );
//     float tr = 1023.0 / t - 1;
//     tr = SERIAL_R / tr;

//   float steinhart;
//   steinhart = tr / THERMISTOR_R; // (R/Ro)
//   steinhart = log(steinhart); // ln(R/Ro)
//   steinhart /= B; // 1/B * ln(R/Ro)
//   steinhart += 1.0 / (NOMINAL_T + 273.15); // +* (1/To)
//   steinhart = 1.0 / steinhart; // Invert
//   steinhart -= 273.15;
//   //Serial.println(steinhart);
//   return steinhart;
// }

void regproc()
{

  regulator.input = t;
  regulator.getResult();
  analogWrite(PWM_PIN, regulator.output);
  TCCR1B = TCCR1B & B11111000 | B00000101;
}

void calproc()
{

  tuner.setInput(t); // передаём текущее значение с датчика. ЖЕЛАТЕЛЬНО ФИЛЬТРОВАННОЕ
  tuner.compute();   // тут производятся вычисления по своему таймеру
  // tuner.getOutput(); // тут можно забрать новый управляющий сигнал
  analogWrite(PWM_PIN, tuner.getOutput()); // например для ШИМ
}

bool wasBadConnection = false;
void loop()
{
  if (enc.tick())
  {
    if (enc.rightH())
    {
      menu = (menu - 1) % 3;
      oled.clear();
    }
    if (enc.leftH())
    {
      menu = (menu + 1) % 3;
      oled.clear();
    }
    if (menu == 0)
    {
      if (enc.right() && (tTemp > 0))
      {
        tTemp--;
        EEPROM.put(EEPROM_ADDRESS, tTemp);
        regulator.setpoint = tTemp;
      }
      if (enc.left() && (tTemp < 100))
      {
        tTemp++;
        EEPROM.put(EEPROM_ADDRESS, tTemp);
        regulator.setpoint = tTemp;
      }
    }
  }
  // static bool flag;
  // static byte val = 0;
  static uint8_t count = 0;
  if (millis() - tmr > POLLING_PERIOD_MS)
  {

    count++;
    if (count == 2)
    {
    count = 0;
    Watchdog.reset();
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
    // Serial.print(millis() / 1000);
    // Serial.print(" ");

    tmr = millis();
    if (!tempproc())
    {
      oled.setCursor(0, 0);
      oled.print("ОШИБКА ТЕРМОДАТЧИКА\r\nПРОВЕРЬТЕ ПОДКЛЮЧЕНИЕ");
      //oled.update();
      analogWrite(PWM_PIN, 0);
      wasBadConnection = true;
      return;
    }
    switch (menu)
    {
    case 0:
      regproc(); // working screen
      // oled.clear();
      if (wasBadConnection)
      {
        wasBadConnection = false;
        oled.clear();
      }
      oled.setCursor(0, 0);
      oled.print("tMES: ");
      oled.print(t);
      oled.setCursor(0, 3);
      oled.print("POW: ");
      oled.print((int)regulator.output * 100 / 255);
      oled.print("%");
      oled.setCursor(0, 6);
      oled.print("tTAR: ");
      oled.print(tTemp);
      oled.setCursor(100, 0);
      oled.print(tmr / 1000);
      // oled.update();
      break;
    case 1:
      regproc(); // tuning screen
      // oled.clear();
      oled.setCursor(20, 0);
      oled.print("T");
      oled.setCursor(0, 0);
      oled.print("Kp: ");
      oled.print(regulator.Kp);
      oled.setCursor(0, 3);
      oled.print("Ki: ");
      oled.print(regulator.Ki);
      oled.setCursor(0, 6);
      oled.print("Kd: ");
      oled.print(regulator.Kd);
      oled.setCursor(120, 0);
      oled.print("T");
      // oled.update();
      break;
    case 2: // cal screen

      // oled.clear();
      oled.setCursor(120, 0);
      oled.print("C");
      if (calACK == 0)
      {
        oled.setCursor(0, 1);
        oled.print("To start PID calibrating procedure, pls hold button");
      }
      else
      {
        calproc();
        oled.clear();
        oled.setCursor(0, 0);
        oled.print("Kp: ");
        oled.print(tuner.getPID_p());
        oled.setCursor(0, 3);
        oled.print("Ki: ");
        oled.print(tuner.getPID_i());
        oled.setCursor(0, 6);
        oled.print("Kd: ");
        oled.print(tuner.getPID_d());
        oled.setCursor(60, 3);
        oled.print("ACC");
        //oled.print(tuner.getAccuracy() > 95);
        delay(10000);
        menu = 0;
      }
      if (enc.held())
      {
        calACK = 1;
      }
      // oled.update();

      break;
    default:
      menu = 0;
    }

    // // Serial.print(regulator.input);
    // //   Serial.print(' ' );
    // //   Serial.println(regulator.output);
    // }
  }
}
