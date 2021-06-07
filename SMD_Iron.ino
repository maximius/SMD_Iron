/*  Скетч для работы подогреваемого стола для пайки SMD-компонентов по термопрофилю.
 * 
 * Для работы скетча необходимо добавить следующие библиотеки:
 * https://alexgyver.ru/gyverpid/
 * https://alexgyver.ru/gyvertimers/
 * 
 * Скетч позволяет отработать работу скетча без подключения датчиков(режим симуляции)
 * Для включения режима симуляции необходимо закомментировать функцию TempCalc() и одновременно раскомментировать функцию proccess();
 * Для работы от датчиков раскомментировать функцию TempCalc() и одновременно закоментировать функцию proccess();
 * 
 * 
 * При нажатии на кнопку UP включается ручной подогрев стола(Удерживается постоянная темпетарура)
 * При нажатии на кнопку DOWN ручной подогрев выключается
 * При нажатии на кнопку SELECT запускается работа по термопрофилю(для правильной работы нужно перезагрузить Arduino)
 * 
 * 
 * PS: Это пока общие наброски программы, и она работает пока неидеально, но в будущем будет дорабатываться.
 * 
 * Что есть:
 * - Включение/выключение ручного режима поддержания температуры стола;
 * - Включение автоматического режима пайки по термопрофилю;
 * - Построение графиков температуры
 * - Режим симуляции (отладка программы без подключения датчиков)
 * 
 * Что планируется реализовать:
 * - Изменение настроек профилей кнопками на Arduino
 * - Сохранение графика термопрофиля для анализа
 * - Сохранение настроек профиля в EEPROM
 * - Выбор профилей для свинцовых и бессвинцовых припоев
 * 
 * 
 * По всем вопросам пишите MaxAltera@yandex.ru
 */



#include <LiquidCrystal.h>
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

#define BTN_UP   1
#define BTN_DOWN 2
#define BTN_LEFT 3
#define BTN_RIGHT 4
#define BTN_SELECT 5
#define BTN_NONE 10

const char *Name[]={"Soaking ","Peak   ","Cooling "};
int set_Time[]={10,20,15};
int set_Temp[]={60,80,0};
int cur_Time=0;
int Tick_Time=0;
int cur_Temp=20;
int i,j;
int TempOk = 0;
int Working=0;
int pinRelay;

#include "GyverTimer.h"   // библиотека таймера
GTimer myTimer(MS);      // создать таймер (по умолч. в режиме интервала)
GTimer clockTimer(MS);      // создать таймер (по умолч. в режиме интервала)
GTimer PidTimer(MS);      // создать таймер (по умолч. в режиме интервала)
GTimer TickTimer(MS); //Таймер времени процесса



//----------------------------Конфигутация термистора------------------------------------------------

// значение 'другого' резистора
 
//#define SERIESRESISTOR 120000
 
// к какому пину подключается термистор
 
#define THERMISTORPIN A1
#define MOSFETPIN 3


#define B 3950 // B-коэффициент
#define SERIAL_R 120000 // сопротивление последовательного резистора, 120 кОм
#define THERMISTOR_R 100000 // номинальное сопротивления термистора, 100 кОм
#define NOMINAL_T 25 // номинальная температура (при которой TR = 100 кОм)

#define NUMSAMPLES 5
int samples[NUMSAMPLES];

void TempCalc() {
 
  uint8_t i;
 
 float average;
 // формируем вектор из N значений с небольшой задержкой между считыванием данных

for (i=0; i< NUMSAMPLES; i++) {
 
samples[i] = analogRead(THERMISTORPIN);
 
delay(10);
 
}
 
// определяем среднее значение в сформированном векторе
 
average = 0;
 
for (i=0; i< NUMSAMPLES; i++) {
 
average += samples[i];
 
}
 
average /= NUMSAMPLES;
 
 

   // int t = analogRead( THERMISTORPIN );
    float tr = 1023.0 / average - 1;
    tr = SERIAL_R / tr;
 //   Serial.print("R=");
 //   Serial.print(tr);
 //   Serial.print(", t=");

    float steinhart;
    steinhart = tr / THERMISTOR_R; // (R/Ro)
    steinhart = log(steinhart); // ln(R/Ro)
    steinhart /= B; // 1/B * ln(R/Ro)
    steinhart += 1.0 / (NOMINAL_T + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart; // Invert
    steinhart -= 273.15; 
    
    cur_Temp=steinhart;
}

//----------------------------------------БЛОК PID Регулятора-----------------------------------------
/*
   Пример работы ПИД регулятора в автоматическом режиме по встроенному таймеру
   Давайте представим, что на 3 пине у нас спираль нагрева, подключенная через мосфет,
   управляем ШИМ сигналом
   И есть какой то абстрактный датчик температуры, на который влияет спираль
*/

// перед подключением библиотеки можно добавить настройки:
// сделает часть вычислений целочисленными, что чуть (совсем чуть!) ускорит код
// #define PID_INTEGER

// режим, при котором интегральная составляющая суммируется только в пределах указанного количества значений
// #define PID_INTEGRAL_WINDOW 50
#include "GyverPID.h"

// GyverPID regulator(4, 0.4, 0.01, 10);  // коэф. П, коэф. И, коэф. Д, период дискретизации dt (мс)
//GyverPID regulator(0.1, 0.05, 0.01, 10);  // коэф. П, коэф. И, коэф. Д, период дискретизации dt (мс)
// или так:
// GyverPID regulator(0.1, 0.05, 0.01);  // можно П, И, Д, без dt, dt будет по умолч. 100 мс


#define DT 100                 //время обновления температуры

#include "GyverPID.h"
//GyverPID regulator(25, 0.03, 0.5 , DT);
GyverPID regulator(1, 0.03, 0.5, DT);

float value = 15;
float signal = 0;
float COEF = 0.01;

//-------------------------------------------------------------------------------


int detectButton() {
  int keyAnalog =  analogRead(A0);
  if (keyAnalog < 100) {
    // Значение меньше 100 – нажата кнопка right
    return BTN_RIGHT;
  } else if (keyAnalog < 200) {
    // Значение больше 100 (иначе мы бы вошли в предыдущий блок результата сравнения, но меньше 200 – нажата кнопка UP
    return BTN_UP;
  } else if (keyAnalog < 400) {
    // Значение больше 200, но меньше 400 – нажата кнопка DOWN
    return BTN_DOWN;
  } else if (keyAnalog < 600) {
    // Значение больше 400, но меньше 600 – нажата кнопка LEFT
    return BTN_LEFT;
  } else if (keyAnalog < 800) {
    // Значение больше 600, но меньше 800 – нажата кнопка SELECT
    return BTN_SELECT;
  } else {
    // Все остальные значения (до 1023) будут означать, что нажатий не было
    return BTN_NONE;
  }
}




void setup() {
  pinMode(MOSFETPIN, OUTPUT);
  Serial.begin(115200);
  lcd.begin(16, 2);
  lcd.print("   SMD Iron");
  delay(2000);
  lcd.setCursor(0, 0);
  lcd.print("  Max Altera");
  delay(2000); 
  lcd.clear();
  i=0;
  drawScreen(0);
  lcd.setCursor(0, 0);
  lcd.print("PressSel");

//--------------------PID регулятор---------------------------
  regulator.setDirection(NORMAL); // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
  regulator.setLimits(0, 255);    // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
  regulator.setpoint = 0;        // сообщаем регулятору температуру, которую он должен поддерживать

  // в процессе работы можно менять коэффициенты
  /*
   regulator.Kp = 5.2;
   regulator.Ki += 0.5;
   regulator.Kd = 0;
   */
//-------------------------------------------------------------
  analogReference(EXTERNAL);
   TempOk=0;
}

//обновляем только оставшееся время
void drawTimer()
   {
    lcd.setCursor(9, 1);
    lcd.print("   ");
    lcd.setCursor(9, 1);
    lcd.print(cur_Time);
    lcd.setCursor(0, 1);
    lcd.print(Tick_Time);
    }

void drawTemp()
   {
    lcd.setCursor(9, 0);
    lcd.print("   ");
    lcd.setCursor(9, 0);
    lcd.print(cur_Temp);
    }    

//пересовываем весь экран
void drawScreen(int i)
   {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(Name[i]);
    lcd.setCursor(0, 1);
    lcd.print(Tick_Time);
    lcd.setCursor(9, 0);
    lcd.print(cur_Temp);
    lcd.setCursor(12, 0);
    lcd.print("/");
    lcd.setCursor(13, 0);
    lcd.print(set_Temp[i]);
    lcd.setCursor(9, 1);
    lcd.print(cur_Time);
    lcd.setCursor(12, 1);
    lcd.print("/");
    lcd.setCursor(13, 1);
    lcd.print(set_Time[i]);
    }

void TimerProcess(int i)          //функция запуска таймера
   {     
   Serial.println("Функция TimerProcess запущена");
 if(!myTimer.isEnabled()&& TempOk==1&& Working==1)         //Если таймер не запущен
   {
//     Serial.println("Температура достигнута");
//     Serial.print("Текущая температура: ");
//     Serial.println(cur_Temp);
//     Serial.print("Установленная температура: ");
//     Serial.println(set_Temp[i]);     
     myTimer.setTimeout(set_Time[i]*1000);
     cur_Time=set_Time[i];
     clockTimer.setTimeout(1000);
     Serial.print("Запущен цикл: ");
     Serial.println(Name[i]);
     drawScreen(i);
    }
     
     if(!myTimer.isEnabled() && TempOk==0)    //Если температура не достигнута
    {
      Serial.println("Запускаем регулировку");
      TickTimer.setTimeout(1000);
      regulator.setpoint=set_Temp[i];    //установлена новая температура

//     Serial.print("Переменная цикла: ");
//     Serial.println(i);
    
    //if(i=0)
    
    if(i=1){                             //если нагрев на втором этапе
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("        ");
    lcd.setCursor(0, 0);
    lcd.print("Heating ");
    lcd.setCursor(12, 0);
    lcd.print("/");
    lcd.setCursor(13, 0);
    lcd.print(set_Temp[i]);
    } else
     {                            //если нагрев на первом этапе
     lcd.clear();
     lcd.setCursor(0, 0);
     lcd.print("        ");
     lcd.setCursor(0, 0);
     lcd.print("PreSoak");
     lcd.setCursor(12, 0);
     lcd.print("/");
     lcd.setCursor(13, 0);
     lcd.print(set_Temp[i]);
    }
     }
   }



void loop() {

  
//читаем, какую кнопку нажали
  int button = detectButton();

  switch (button) {
    case BTN_UP:
     // printDisplay("UP");
       if(Working==0)    //Если не запущен автоматический режим
    {
       regulator.setpoint=50;    //включаем ручной подогрев стола
       lcd.setCursor(0, 0);
       lcd.print("MANU ON ");
       lcd.setCursor(13, 0);
       lcd.print(" 50");
       Serial.println("Ручной режим включен");
    }
      break;
    case BTN_DOWN:
     // printDisplay("DOWN");
      if(Working==0)    //Если не запущен автоматический режим
    {
       regulator.setpoint=0;    //выключаем ручной подогрев стола
       lcd.setCursor(0, 0);
       lcd.print("MANU OFF");
       lcd.setCursor(13, 0);
       lcd.print("0  ");
       Serial.println("Ручной режим выключен");
    }
      break;
    case BTN_LEFT:
      //printDisplay("LEFT");
      break;
    case BTN_RIGHT:
      //printDisplay("RIGHT");
      break;
    case BTN_SELECT:
      //printDisplay("SELECT");
//      Serial.println("Кнопка нажата");
      i=0;
      Working=1;
      TimerProcess(i);
      break;      
    default:
      //printDisplay("Press any key");
      break;
      
  }

//---------------------PID регулятор---------------------------------
 /* regulator.input = temp;   // сообщаем регулятору текущую температуру

  // getResultTimer возвращает значение для управляющего устройства
  // (после вызова можно получать это значение как regulator.output)
  // обновление происходит по встроенному таймеру на millis()
  analogWrite(3, regulator.getResultTimer());  // отправляем на мосфет

  // .getResultTimer() по сути возвращает regulator.output

*/

//Вычитаем секунды из заданного времени, если запущен таймер 
  if (clockTimer.isReady()& myTimer.isEnabled()){
//         Serial.println("Сработал таймер ClockTimer");
     //if (i>0){
          cur_Time--;
          clockTimer.setTimeout(1000);
//          Serial.println("Отсчет -1");
          //Tick_Time++;
          drawTimer();
      //}  
     }

   if (TickTimer.isReady()&&Working==1){
         //Serial.println("Сработал таймер TickTimer");
          Tick_Time++;
          drawTimer();
          TickTimer.setTimeout(1000);
      //}  
     }
 
 
 
/* if (PidTimer.isReady() && TempOk==0){
     Serial.println("Сработал таймер PidTimer");
     Serial.print("Текущая температура: ");
     Serial.println(cur_Temp);
     Serial.print("Установленная температура: ");
     Serial.println(set_Temp[i]);     
        
     regulator.input = cur_Temp;   // сообщаем регулятору текущую температуру

  // getResultTimer возвращает значение для управляющего устройства
  // (после вызова можно получать это значение как regulator.output)
  // обновление происходит по встроенному таймеру на millis()
     //analogWrite(3, regulator.getResultTimer());  // Обновляем новое значение MOSFET

  // .getResultTimer() по сути возвращает regulator.output
      PidTimer.setTimeout(1000);
 } */

  if  ((cur_Temp > set_Temp[i]*0.99) && TempOk==0)       //Если температура достигла 95% от заданной, запускаем таймер
      {
        Serial.println("Температура установлена");
        TempOk=1;
        TimerProcess(i);
        }
/*
  if (signal>10){         //работаем в ключевом режиме для реле
    pinRelay=100;      //для отображения на симуляции
    digitalWrite(MOSFETPIN, HIGH);
  } else
   {
    pinRelay=0;        //для отображения на симуляции
    digitalWrite(MOSFETPIN, LOW);
    }
*/

digitalWrite(MOSFETPIN, signal);   //работаем в ШИМ режиме

    
  //Ловим окончание цикла
 
  if (myTimer.isReady()){               //если цикл закончился, то переходим к следующему
     if (i<2){
        Serial.print("Конец цикла: ");
        Serial.println(Name[i]);
        i++;
        TempOk=0;
        TimerProcess(i);
     }
     else {                             //иначе заканчиваем работу
        lcd.setCursor(0, 0);
        Working=0;
        lcd.print("FINISH! ");
        Serial.println("Конец работы");
        i=0;
        TempOk=0;
       }
      }
     
     //-------------------------------Блок симуляции и регулирования----------------------------------------

      // процесс и графики
  static uint32_t tmr;
  if (millis() - tmr >= DT) {
    drawTemp();
    tmr = millis();

    //TempCalc();       //текущая температура с датчика(выключить при симуляции)
   
    
    regulator.input = cur_Temp;
    signal = regulator.getResult();
    if (regulator.integral < 0) regulator.integral = 0;
    
   
    process();        //вызывать при симуляции



    Serial.print(cur_Temp); Serial.print(' ');
    Serial.print(signal); Serial.print(' ');
    Serial.print(pinRelay); Serial.print(' ');
    //Serial.print(regulator.integral); Serial.print(' ');
    Serial.println(regulator.setpoint);
    
  }

  // настройка
  if (Serial.available() > 1) {
    char incoming = Serial.read();
    float value = Serial.parseFloat();
    switch (incoming) {
      case 'p': regulator.Kp = value;
        break;
      case 'i': regulator.Ki = value;
        break;
      case 'd': regulator.Kd = value;
        break;
      case 's': regulator.setpoint = value;
        break;
      case 'c': COEF = value;
        break;
    }
  }
     
     
     
     
     }


//------------------------Симуляция процесса

#define LOW_VALUE 15      // к этому значению "остывает" система
#define SIGNAL_COEF 0.1   // сила сигнала
#define DELAY_AMOUNT 5   // задержка изменения

     void process() {
  static float valueSpeed;
  static float signalSpeed;
  static bool firstFlag = false;
  static float delayArray[DELAY_AMOUNT];
  
  // сигнал == скорость нагрева
  // ограничивает сигнал его же значением и плавно к нему стремится
  signalSpeed += (signal - signalSpeed) * 0.02;

  // складываем скорость сигнала и скорость охлаждения
  // скорость охлаждения получаем как разность "температуры" и её нулевого значения
  valueSpeed = signalSpeed * SIGNAL_COEF + (LOW_VALUE - cur_Temp) * COEF;

  if (!firstFlag) {
    firstFlag = true;
    for (int i = 0; i < DELAY_AMOUNT; i++) delayArray[i] = valueSpeed;
  }

  for (int i = 0; i < DELAY_AMOUNT - 1; i++) delayArray[i] = delayArray[i + 1];
  delayArray[DELAY_AMOUNT - 1] = valueSpeed;

  // прибавляем скорость (интегрируем)
  cur_Temp +=/* valueSpeed*/delayArray[0];
}
