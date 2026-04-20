 #include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <SoftwareSerial.h>

#define LCD_ADDR            0x27
#define PUMP1_PIN           10
#define PUMP2_PIN           11
#define ESP_RX_PIN          A2
#define ESP_TX_PIN          A3
#define SERIAL_BAUD         115200
#define ESP_BAUD            9600

#define PUMP_ON_LEVEL       HIGH
#define PUMP_OFF_LEVEL      LOW
#define PUMP1_PWM_PERCENT   90
#define PUMP2_PWM_PERCENT   80
#define PUMP1_PWM_VALUE     ((255 * PUMP1_PWM_PERCENT) / 100)
#define PUMP2_PWM_VALUE     ((255 * PUMP2_PWM_PERCENT) / 100)

#define LCD_UPDATE_MS       200UL
#define LCD_SWITCH_MS       3000UL
#define RESUME_MSG_MS       1000UL
#define AUTO_WAIT_MS        7200000UL
#define RUN_P1_MS           720000UL   // 12 min 00 sec
#define RUN_P2_MS           680000UL   // 11 min 20 sec

#define SRC_KEYPAD          "KEYPAD"
#define SRC_BLYNK           "BLYNK"
#define SRC_ESP             "ESP"
#define SRC_UNO             "UNO"

#define ACT_SINGLE_P1       "SINGLE_P1"
#define ACT_SINGLE_P2       "SINGLE_P2"
#define ACT_SINGLE_OFF      "SINGLE_OFF"
#define ACT_MANUAL_ON       "MANUAL_ON"
#define ACT_MANUAL_OFF      "MANUAL_OFF"
#define ACT_AUTO_ON         "AUTO_ON"
#define ACT_AUTO_OFF        "AUTO_OFF"
#define ACT_PAUSE_ON        "PAUSE_ON"
#define ACT_PAUSE_OFF       "PAUSE_OFF"
#define ACT_STOP_ON         "STOP_ON"
#define ACT_STOP_OFF        "STOP_OFF"
#define ACT_STATUS_REQ      "STATUS_REQ"

#define EVT_BOOT            "BOOT"
#define EVT_P1_ON           "PUMP1_ON"
#define EVT_P2_ON           "PUMP2_ON"
#define EVT_SINGLE_OFF      "SINGLE_OFF"
#define EVT_MANUAL_ON       "MANUAL_ON"
#define EVT_MANUAL_OFF      "MANUAL_OFF"
#define EVT_AUTO_ON         "AUTO_ON"
#define EVT_AUTO_OFF        "AUTO_OFF"
#define EVT_AUTO_RUN        "AUTO_STAGE_RUN"
#define EVT_AUTO_WAIT       "AUTO_STAGE_WAIT"
#define EVT_TIMER_SET       "TIMER_SET"
#define EVT_TIMER_RUN       "TIMER_RUN"
#define EVT_TIMER_DONE      "TIMER_DONE"
#define EVT_PAUSE_ON        "PAUSE_ON"
#define EVT_PAUSE_OFF       "PAUSE_OFF"
#define EVT_STOP_ON         "STOP_ON"
#define EVT_STOP_OFF        "STOP_OFF"
#define EVT_STATUS          "STATUS"

LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
SoftwareSerial espSerial(ESP_RX_PIN, ESP_TX_PIN);

const byte ROWS = 4;
const byte COLS = 4;

char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

byte rowPins[ROWS] = {9, 8, 7, 6};
byte colPins[COLS] = {5, 4, 3, 2};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

enum State
{
  ST_HOME,
  ST_SET_TIME,
  ST_WAIT_TIME,
  ST_TIMER_RUN,
  ST_AUTO_CONFIRM,
  ST_AUTO_WAIT,
  ST_AUTO_RUN,
  ST_MANUAL
};

State currentState = ST_HOME;
State savedStateBeforePause = ST_HOME;

bool pauseActive = false;
bool stopLatched = false;

bool singleP1 = false;
bool singleP2 = false;
bool savedSingleP1 = false;
bool savedSingleP2 = false;

bool clockValid = false;
byte nowHour = 0;
byte nowMinute = 0;
byte nowSecond = 0;

bool targetValid = false;
byte targetHour = 0;
byte targetMinute = 0;
byte targetSecond = 0;

char setDigits[7];
byte setLen = 0;

unsigned long stateStartMs = 0;
unsigned long pauseElapsedMs = 0;
unsigned long lastClockTickMs = 0;
unsigned long lastLcdMs = 0;
unsigned long lastLcdSwitchMs = 0;
unsigned long resumeMsgUntilMs = 0;
byte lcdPage = 0;

char rxBuf[96];
byte rxIdx = 0;

char lcd1[17];
char lcd2[17];

void clear16(char *buf)
{
  for (byte i = 0; i < 16; i++) buf[i] = ' ';
  buf[16] = '\0';
}

void copyTo16(char *dst, const char *src)
{
  byte i = 0;
  while (i < 16 && src[i] != '\0')
  {
    dst[i] = src[i];
    i++;
  }
  while (i < 16)
  {
    dst[i] = ' ';
    i++;
  }
  dst[16] = '\0';
}

void centerTo16(char *dst, const char *src)
{
  clear16(dst);
  byte len = strlen(src);
  if (len > 16) len = 16;
  byte start = (16 - len) / 2;
  for (byte i = 0; i < len; i++) dst[start + i] = src[i];
}

void print2d(byte v)
{
  if (v < 10) lcd.print('0');
  lcd.print(v);
}

void formatTimeText(char *buf, byte hh, byte mm, byte ss)
{
  snprintf(buf, 9, "%02u:%02u:%02u", hh, mm, ss);
}

void formatBarTime(char *buf, byte hh, byte mm, byte ss)
{
  snprintf(buf, 9, "%02u|%02u|%02u", hh, mm, ss);
}

void getNowText(char *buf)
{
  if (!clockValid)
  {
    strcpy(buf, "--:--:--");
    return;
  }
  formatTimeText(buf, nowHour, nowMinute, nowSecond);
}

void getTargetText(char *buf)
{
  if (!targetValid)
  {
    strcpy(buf, "--|--|--");
    return;
  }
  formatBarTime(buf, targetHour, targetMinute, targetSecond);
}

void getInputText(char *buf)
{
  char raw[7] = "------";
  for (byte i = 0; i < 6; i++)
  {
    if (i < setLen) raw[i] = setDigits[i];
  }
  raw[6] = '\0';
  snprintf(buf, 9, "%c%c|%c%c|%c%c", raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]);
}

bool timedRunWindowActive(unsigned long runMs)
{
  if (!(currentState == ST_AUTO_RUN || currentState == ST_TIMER_RUN)) return false;
  return (millis() - stateStartMs) < runMs;
}

bool pump1On()
{
  if (stopLatched || pauseActive) return false;
  if (currentState == ST_MANUAL) return true;
  if (timedRunWindowActive(RUN_P1_MS)) return true;
  return singleP1;
}

bool pump2On()
{
  if (stopLatched || pauseActive) return false;
  if (currentState == ST_MANUAL) return true;
  if (timedRunWindowActive(RUN_P2_MS)) return true;
  return singleP2;
}

const char* getModeText()
{
  if (stopLatched) return "STOP";
  if (pauseActive) return "PAUSE";

  switch (currentState)
  {
    case ST_MANUAL:       return "MANUAL";
    case ST_AUTO_CONFIRM:
    case ST_AUTO_WAIT:
    case ST_AUTO_RUN:     return "AUTO";
    case ST_WAIT_TIME:
    case ST_TIMER_RUN:    return "TIMER";
    default:              return "NONE";
  }
}

void applyPumps()
{
  analogWrite(PUMP1_PIN, pump1On() ? PUMP1_PWM_VALUE : 0);
  analogWrite(PUMP2_PIN, pump2On() ? PUMP2_PWM_VALUE : 0);
}

void clearDigits()
{
  setLen = 0;
  setDigits[0] = '\0';
}

void clearSingles()
{
  singleP1 = false;
  singleP2 = false;
}

void resetTimerTarget()
{
  targetValid = false;
  targetHour = 0;
  targetMinute = 0;
  targetSecond = 0;
}

void setClock(byte hh, byte mm, byte ss)
{
  if (hh > 23 || mm > 59 || ss > 59) return;
  nowHour = hh;
  nowMinute = mm;
  nowSecond = ss;
  clockValid = true;
  lastClockTickMs = millis();
}

void tickClock()
{
  if (!clockValid) return;

  nowSecond++;
  if (nowSecond >= 60)
  {
    nowSecond = 0;
    nowMinute++;
    if (nowMinute >= 60)
    {
      nowMinute = 0;
      nowHour++;
      if (nowHour >= 24) nowHour = 0;
    }
  }
}

void updateSoftClock()
{
  if (!clockValid) return;

  while (millis() - lastClockTickMs >= 1000UL)
  {
    lastClockTickMs += 1000UL;
    tickClock();
  }
}

void goHome(bool clearTarget)
{
  currentState = ST_HOME;
  pauseActive = false;
  stopLatched = false;
  stateStartMs = millis();
  pauseElapsedMs = 0;
  lcdPage = 0;
  lastLcdSwitchMs = millis();
  clearDigits();
  clearSingles();

  if (clearTarget) resetTimerTarget();

  applyPumps();
}

void enterState(State s)
{
  currentState = s;
  stateStartMs = millis();
  lcdPage = 0;
  lastLcdSwitchMs = millis();
  applyPumps();
}

bool inputValid()
{
  if (setLen != 6) return false;

  int hh = (setDigits[0] - '0') * 10 + (setDigits[1] - '0');
  int mm = (setDigits[2] - '0') * 10 + (setDigits[3] - '0');
  int ss = (setDigits[4] - '0') * 10 + (setDigits[5] - '0');

  if (hh < 0 || hh > 23) return false;
  if (mm < 0 || mm > 59) return false;
  if (ss < 0 || ss > 59) return false;

  return true;
}

void saveInputToTarget()
{
  targetHour   = (setDigits[0] - '0') * 10 + (setDigits[1] - '0');
  targetMinute = (setDigits[2] - '0') * 10 + (setDigits[3] - '0');
  targetSecond = (setDigits[4] - '0') * 10 + (setDigits[5] - '0');
  targetValid = true;
}

void safeText16(char *text)
{
  for (byte i = 0; i < 16 && text[i] != '\0'; i++)
  {
    if (text[i] == ';') text[i] = '/';
    if (text[i] == '=') text[i] = '-';
  }
}

void sendState(const char *src, const char *evt)
{
  char l1[17];
  char l2[17];

  copyTo16(l1, lcd1);
  copyTo16(l2, lcd2);
  safeText16(l1);
  safeText16(l2);

  espSerial.print(F("STATE:SRC="));
  espSerial.print(src);
  espSerial.print(F(";EVENT="));
  espSerial.print(evt);
  espSerial.print(F(";MODE="));
  espSerial.print(getModeText());
  espSerial.print(F(";P1="));
  espSerial.print(pump1On() ? 1 : 0);
  espSerial.print(F(";P2="));
  espSerial.print(pump2On() ? 1 : 0);
  espSerial.print(F(";PAUSE="));
  espSerial.print(pauseActive ? 1 : 0);
  espSerial.print(F(";STOP="));
  espSerial.print(stopLatched ? 1 : 0);
  espSerial.print(F(";L1="));
  espSerial.print(l1);
  espSerial.print(F(";L2="));
  espSerial.println(l2);
}


void buildPumpStatusText(char *buf)
{
  const char *p1 = pump1On() ? "ON " : "OFF";
  const char *p2 = pump2On() ? "ON " : "OFF";
  char tmp[17];
  snprintf(tmp, 17, "P1-%s|P2-%s", p1, p2);
  centerTo16(buf, tmp);
}

void buildHomeTimeLine(char *buf)
{
  char t[9];
  char tmp[17];
  getNowText(t);
  snprintf(tmp, 17, "TIME %s", t);
  centerTo16(buf, tmp);
}

void buildHomeStatusLine(char *buf)
{
  bool p1 = pump1On();
  bool p2 = pump2On();

  if (!p1 && !p2)
  {
    centerTo16(buf, "READY TO RUN");
  }
  else if (p1 && p2)
  {
    centerTo16(buf, "2 PUMPS ACTIVE");
  }
  else
  {
    centerTo16(buf, "1 PUMP ACTIVE");
  }
}

void buildNowLine(char *buf)
{
  char t[9];
  char tmp[17];
  getNowText(t);
  snprintf(tmp, 17, "NOW %s", t);
  centerTo16(buf, tmp);
}

void buildAutoWaitLine(char *buf)
{
  unsigned long remainMs = 0;
  if (millis() - stateStartMs < AUTO_WAIT_MS) remainMs = AUTO_WAIT_MS - (millis() - stateStartMs);

  unsigned long totalSec = remainMs / 1000UL;
  unsigned int hh = totalSec / 3600UL;
  unsigned int mm = (totalSec % 3600UL) / 60UL;
  unsigned int ss = totalSec % 60UL;

  char tmp[17];
  snprintf(tmp, sizeof(tmp), "IN %02u:%02u:%02u", hh, mm, ss);
  centerTo16(buf, tmp);
}

void buildRunLeftLine(char *buf)
{
  unsigned long totalRunMs = (RUN_P1_MS > RUN_P2_MS) ? RUN_P1_MS : RUN_P2_MS;
  unsigned long remainMs = 0;
  if (millis() - stateStartMs < totalRunMs) remainMs = totalRunMs - (millis() - stateStartMs);

  unsigned long totalSec = remainMs / 1000UL;
  unsigned int mm = totalSec / 60UL;
  unsigned int ss = totalSec % 60UL;

  snprintf(buf, 17, "LEFT %02u:%02u:%02u", 0U, mm, ss);
}

void buildPumpLeftLine(char *buf, byte pumpNo, unsigned long runMs)
{
  unsigned long elapsedMs = millis() - stateStartMs;

  if (elapsedMs >= runMs)
  {
    snprintf(buf, 17, "P%u OFF EARLY", pumpNo);
    return;
  }

  unsigned long remainMs = runMs - elapsedMs;
  unsigned long totalSec = remainMs / 1000UL;
  unsigned int mm = totalSec / 60UL;
  unsigned int ss = totalSec % 60UL;

  snprintf(buf, 17, "P%u LEFT %02u:%02u:%02u", pumpNo, 0U, mm, ss);
}

void updateLcdPageFlip(bool allowFlip)
{
  if (!allowFlip)
  {
    lcdPage = 0;
    lastLcdSwitchMs = millis();
    return;
  }

  if (millis() - lastLcdSwitchMs >= LCD_SWITCH_MS)
  {
    lastLcdSwitchMs = millis();
    lcdPage = (lcdPage == 0) ? 1 : 0;
  }
}

void updateLcdCache()
{
  char buf1[17];
  char buf2[17];
  char t[9];
  bool allowFlip = false;

  clear16(buf1);
  clear16(buf2);

  if (resumeMsgUntilMs > millis())
  {
    centerTo16(buf1, "SYSTEM RESUME");
    centerTo16(buf2, "RUN CONTINUE");
  }
  else if (stopLatched)
  {
    centerTo16(buf1, "EMERGENCY STOP");
    centerTo16(buf2, "ALL PUMPS OFF");
  }
  else if (!clockValid && (currentState == ST_HOME || currentState == ST_WAIT_TIME))
  {
    centerTo16(buf1, "TIME NOT READY");
    centerTo16(buf2, "WAIT SYNC...");
  }
  else if (pauseActive)
  {
    centerTo16(buf1, "MODE: PAUSE");
    buildPumpStatusText(buf2);
  }
  else if (currentState == ST_HOME)
  {
    allowFlip = true;
    if (lcdPage == 0)
    {
      buildHomeTimeLine(buf1);
      buildPumpStatusText(buf2);
    }
    else
    {
      centerTo16(buf1, "MODE: NORMAL");
      buildHomeStatusLine(buf2);
    }
  }
  else if (currentState == ST_SET_TIME)
  {
    getInputText(t);
    if (setLen >= 6)
    {
      centerTo16(buf1, "CONFIRM TIME");
      centerTo16(buf2, t);
    }
    else
    {
      centerTo16(buf1, "SET START TIME");
      centerTo16(buf2, t);
    }
  }
  else if (currentState == ST_WAIT_TIME)
  {
    char targetBar[9];
    char targetClock[9];
    char atLine[17];
    getTargetText(targetBar);
    formatTimeText(targetClock, targetHour, targetMinute, targetSecond);

    allowFlip = true;
    if (lcdPage == 0)
    {
      centerTo16(buf1, "WAIT START");
      snprintf(atLine, 17, "AT %s", targetClock);
      centerTo16(buf2, atLine);
    }
    else
    {
      centerTo16(buf1, "WAITING TIMER");
      buildNowLine(buf2);
    }
  }
  else if (currentState == ST_TIMER_RUN)
  {
    allowFlip = true;
    if (lcdPage == 0)
    {
      char leftLine[17];
      buildRunLeftLine(leftLine);
      copyTo16(buf1, "TIMER RUNNING");
      copyTo16(buf2, leftLine);
    }
    else
    {
      buildPumpLeftLine(buf1, 1, RUN_P1_MS);
      buildPumpLeftLine(buf2, 2, RUN_P2_MS);
    }
  }
  else if (currentState == ST_AUTO_CONFIRM)
  {
    centerTo16(buf1, "AUTO MODE SET");
    centerTo16(buf2, "PRESS D TO OK");
  }
  else if (currentState == ST_AUTO_WAIT)
  {
    allowFlip = true;
    if (lcdPage == 0)
    {
      centerTo16(buf1, "AUTO NEXT RUN");
      buildAutoWaitLine(buf2);
    }
    else
    {
      centerTo16(buf1, "MODE: AUTO");
      centerTo16(buf2, "CYCLE ACTIVE");
    }
  }
  else if (currentState == ST_AUTO_RUN)
  {
    allowFlip = true;
    if (lcdPage == 0)
    {
      centerTo16(buf1, "AUTO RUN");
      buildRunLeftLine(buf2);
    }
    else
    {
      buildPumpLeftLine(buf1, 1, RUN_P1_MS);
      buildPumpLeftLine(buf2, 2, RUN_P2_MS);
    }
  }
  else if (currentState == ST_MANUAL)
  {
    allowFlip = true;
    if (lcdPage == 0)
    {
      centerTo16(buf1, "MODE: MANUAL");
      buildPumpStatusText(buf2);
    }
    else
    {
      centerTo16(buf1, "MODE: MANUAL");
      centerTo16(buf2, "SYSTEM ACTIVE");
    }
  }

  updateLcdPageFlip(allowFlip);

  copyTo16(lcd1, buf1);
  copyTo16(lcd2, buf2);
}

void updateLCD()
{
  if (millis() - lastLcdMs < LCD_UPDATE_MS) return;
  lastLcdMs = millis();

  updateLcdCache();
  lcd.setCursor(0, 0);
  lcd.print(lcd1);
  lcd.setCursor(0, 1);
  lcd.print(lcd2);
}

void showInvalidTime()
{
  lcd.setCursor(0, 0);
  lcd.print(F(" Invalid Time   "));
  lcd.setCursor(0, 1);
  lcd.print(F(" Need hhmmss    "));
  delay(700);
}

void keypadStopNow()
{
  currentState = ST_HOME;
  pauseActive = false;
  stopLatched = false;
  clearDigits();
  clearSingles();
  resetTimerTarget();
  lcdPage = 0;
  lastLcdSwitchMs = millis();
  applyPumps();
  sendState(SRC_KEYPAD, "STOP_KEY");
}

void pauseOn(const char *src)
{
  if (stopLatched || pauseActive) return;

  pauseActive = true;
  savedStateBeforePause = currentState;
  savedSingleP1 = singleP1;
  savedSingleP2 = singleP2;
  pauseElapsedMs = millis() - stateStartMs;

  applyPumps();
  sendState(src, EVT_PAUSE_ON);
}

void pauseOff(const char *src)
{
  if (!pauseActive) return;

  pauseActive = false;
  currentState = savedStateBeforePause;
  singleP1 = savedSingleP1;
  singleP2 = savedSingleP2;
  stateStartMs = millis() - pauseElapsedMs;
  pauseElapsedMs = 0;
  resumeMsgUntilMs = millis() + RESUME_MSG_MS;
  lcdPage = 0;
  lastLcdSwitchMs = millis();

  applyPumps();
  sendState(src, EVT_PAUSE_OFF);
}

void stopOn(const char *src)
{
  stopLatched = true;
  pauseActive = false;
  currentState = ST_HOME;
  clearDigits();
  clearSingles();
  resetTimerTarget();
  lcdPage = 0;
  lastLcdSwitchMs = millis();
  applyPumps();
  sendState(src, EVT_STOP_ON);
}

void stopOff(const char *src)
{
  stopLatched = false;
  currentState = ST_HOME;
  clearDigits();
  clearSingles();
  resetTimerTarget();
  lcdPage = 0;
  lastLcdSwitchMs = millis();
  applyPumps();
  sendState(src, EVT_STOP_OFF);
}

void handleDigit(char key)
{
  if (pauseActive || stopLatched) return;
  if (currentState != ST_SET_TIME) return;

  if (setLen < 6)
  {
    setDigits[setLen++] = key;
    setDigits[setLen] = '\0';
  }
}

void handleHash()
{
  if (pauseActive || stopLatched) return;

  if (currentState == ST_HOME || currentState == ST_WAIT_TIME)
  {
    clearDigits();
    clearSingles();
    enterState(ST_SET_TIME);
  }
}

void handleStar()
{
  if (pauseActive || stopLatched) return;
  if (currentState == ST_SET_TIME) clearDigits();
}

void handleA()
{
  if (pauseActive || stopLatched) return;

  if (currentState == ST_HOME)
  {
    clearSingles();
    resetTimerTarget();
    enterState(ST_AUTO_CONFIRM);
  }
}

void handleB()
{
  if (pauseActive || stopLatched) return;

  if (currentState == ST_HOME)
  {
    clearSingles();
    resetTimerTarget();
    enterState(ST_MANUAL);
    sendState(SRC_KEYPAD, EVT_MANUAL_ON);
  }
}

void handleD()
{
  if (pauseActive || stopLatched) return;

  if (currentState == ST_SET_TIME)
  {
    if (!inputValid())
    {
      showInvalidTime();
      return;
    }

    saveInputToTarget();
    clearDigits();
    clearSingles();
    enterState(ST_WAIT_TIME);
    sendState(SRC_KEYPAD, EVT_TIMER_SET);
    return;
  }

  if (currentState == ST_AUTO_CONFIRM)
  {
    clearSingles();
    resetTimerTarget();
    enterState(ST_AUTO_WAIT);
    sendState(SRC_KEYPAD, EVT_AUTO_ON);
    return;
  }
}

void handleKeypad()
{
  char key = keypad.getKey();
  if (!key) return;

  if (key == 'C')
  {
    keypadStopNow();
    return;
  }

  if (key >= '0' && key <= '9')
  {
    handleDigit(key);
    return;
  }

  if (key == '#') handleHash();
  else if (key == '*') handleStar();
  else if (key == 'A') handleA();
  else if (key == 'B') handleB();
  else if (key == 'D') handleD();
}

void doSingleP1(const char *src)
{
  if (pauseActive || stopLatched) return;

  currentState = ST_HOME;
  clearDigits();
  resetTimerTarget();
  singleP1 = true;
  singleP2 = false;
  applyPumps();
  sendState(src, EVT_P1_ON);
}

void doSingleP2(const char *src)
{
  if (pauseActive || stopLatched) return;

  currentState = ST_HOME;
  clearDigits();
  resetTimerTarget();
  singleP1 = false;
  singleP2 = true;
  applyPumps();
  sendState(src, EVT_P2_ON);
}

void doSingleOff(const char *src)
{
  if (pauseActive || stopLatched) return;

  currentState = ST_HOME;
  clearDigits();
  resetTimerTarget();
  clearSingles();
  applyPumps();
  sendState(src, EVT_SINGLE_OFF);
}

void doManualOn(const char *src)
{
  if (pauseActive || stopLatched) return;

  clearSingles();
  resetTimerTarget();
  enterState(ST_MANUAL);
  sendState(src, EVT_MANUAL_ON);
}

void doManualOff(const char *src)
{
  if (pauseActive || stopLatched) return;

  goHome(true);
  sendState(src, EVT_MANUAL_OFF);
}

void doAutoOn(const char *src)
{
  if (pauseActive || stopLatched) return;

  clearSingles();
  resetTimerTarget();
  enterState(ST_AUTO_WAIT);
  sendState(src, EVT_AUTO_ON);
}

void doAutoOff(const char *src)
{
  if (pauseActive || stopLatched) return;

  goHome(true);
  sendState(src, EVT_AUTO_OFF);
}

void handleAction(const char *src, const char *act)
{
  if (strcmp(act, ACT_STATUS_REQ) == 0)
  {
    sendState(SRC_UNO, EVT_STATUS);
    return;
  }

  if (strcmp(act, ACT_STOP_ON) == 0)
  {
    stopOn(src);
    return;
  }

  if (strcmp(act, ACT_STOP_OFF) == 0)
  {
    stopOff(src);
    return;
  }

  if (stopLatched)
  {
    sendState(SRC_UNO, EVT_STATUS);
    return;
  }

  if (strcmp(act, ACT_PAUSE_ON) == 0)
  {
    pauseOn(src);
    return;
  }

  if (strcmp(act, ACT_PAUSE_OFF) == 0)
  {
    pauseOff(src);
    return;
  }

  if (pauseActive)
  {
    sendState(SRC_UNO, EVT_STATUS);
    return;
  }

  if (strcmp(act, ACT_SINGLE_P1) == 0)
  {
    doSingleP1(src);
    return;
  }

  if (strcmp(act, ACT_SINGLE_P2) == 0)
  {
    doSingleP2(src);
    return;
  }

  if (strcmp(act, ACT_SINGLE_OFF) == 0)
  {
    doSingleOff(src);
    return;
  }

  if (strcmp(act, ACT_MANUAL_ON) == 0)
  {
    doManualOn(src);
    return;
  }

  if (strcmp(act, ACT_MANUAL_OFF) == 0)
  {
    doManualOff(src);
    return;
  }

  if (strcmp(act, ACT_AUTO_ON) == 0)
  {
    doAutoOn(src);
    return;
  }

  if (strcmp(act, ACT_AUTO_OFF) == 0)
  {
    doAutoOff(src);
    return;
  }

  sendState(SRC_UNO, EVT_STATUS);
}

bool parseTimeLine(const char *line)
{
  if (strncmp(line, "TIME:", 5) != 0) return false;

  int hh = -1;
  int mm = -1;
  int ss = -1;

  if (sscanf(line + 5, "%d:%d:%d", &hh, &mm, &ss) != 3) return false;
  if (hh < 0 || hh > 23 || mm < 0 || mm > 59 || ss < 0 || ss > 59) return false;

  setClock((byte)hh, (byte)mm, (byte)ss);
  return true;
}

void parseCmdLine(char *line)
{
  if (parseTimeLine(line)) return;
  if (strncmp(line, "CMD:", 4) != 0) return;

  char src[12] = "ESP";
  char act[18] = "";

  char *pSrc = strstr(line, "SRC=");
  char *pAct = strstr(line, "ACT=");

  if (pSrc)
  {
    pSrc += 4;
    byte i = 0;
    while (*pSrc && *pSrc != ';' && i < sizeof(src) - 1)
    {
      src[i++] = *pSrc++;
    }
    src[i] = '\0';
  }

  if (pAct)
  {
    pAct += 4;
    byte i = 0;
    while (*pAct && *pAct != ';' && i < sizeof(act) - 1)
    {
      act[i++] = *pAct++;
    }
    act[i] = '\0';
  }

  handleAction(src, act);
}

void readEspSerial()
{
  while (espSerial.available())
  {
    char c = espSerial.read();

    if (c == '\r') continue;

    if (c == '\n')
    {
      rxBuf[rxIdx] = '\0';
      if (rxIdx > 0) parseCmdLine(rxBuf);
      rxIdx = 0;
    }
    else
    {
      if (rxIdx < sizeof(rxBuf) - 1)
      {
        rxBuf[rxIdx++] = c;
      }
      else
      {
        rxIdx = 0;
      }
    }
  }
}

void updateLogic()
{
  if (stopLatched || pauseActive)
  {
    applyPumps();
    return;
  }

  if (currentState == ST_WAIT_TIME && targetValid && clockValid)
  {
    if (nowHour == targetHour && nowMinute == targetMinute && nowSecond == targetSecond)
    {
      enterState(ST_TIMER_RUN);
      sendState(SRC_UNO, EVT_TIMER_RUN);
    }
  }

  if (currentState == ST_TIMER_RUN)
  {
    unsigned long totalRunMs = (RUN_P1_MS > RUN_P2_MS) ? RUN_P1_MS : RUN_P2_MS;
    if (millis() - stateStartMs >= totalRunMs)
    {
      goHome(true);
      sendState(SRC_UNO, EVT_TIMER_DONE);
    }
  }

  if (currentState == ST_AUTO_WAIT)
  {
    if (millis() - stateStartMs >= AUTO_WAIT_MS)
    {
      enterState(ST_AUTO_RUN);
      sendState(SRC_UNO, EVT_AUTO_RUN);
    }
  }

  if (currentState == ST_AUTO_RUN)
  {
    unsigned long totalRunMs = (RUN_P1_MS > RUN_P2_MS) ? RUN_P1_MS : RUN_P2_MS;
    if (millis() - stateStartMs >= totalRunMs)
    {
      enterState(ST_AUTO_WAIT);
      sendState(SRC_UNO, EVT_AUTO_WAIT);
    }
  }

  applyPumps();
}

void setup()
{
  pinMode(PUMP1_PIN, OUTPUT);
  pinMode(PUMP2_PIN, OUTPUT);
  analogWrite(PUMP1_PIN, 0);
  analogWrite(PUMP2_PIN, 0);

  Serial.begin(SERIAL_BAUD);
  espSerial.begin(ESP_BAUD);

  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Air Mattress    "));
  lcd.setCursor(0, 1);
  lcd.print(F("Demo System     "));
  delay(1200);

  goHome(true);
  updateLcdCache();
  updateLCD();
  sendState(SRC_UNO, EVT_BOOT);
}

void loop()
{
  readEspSerial();
  updateSoftClock();
  handleKeypad();
  updateLogic();
  updateLCD();
}
