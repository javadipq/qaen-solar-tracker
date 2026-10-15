#include <Wire.h>
#include "RTClib.h"
#include <math.h>

RTC_DS3231 rtc;

// ================== تنظیمات مختصات جغرافیایی قاین ==================
const double LATITUDE  = 33.7275;  // عرض جغرافیایی قاین (شمالی)
const double LONGITUDE = 59.1844;  // طول جغرافیایی قاین (شرقی)
const double TIMEZONE  = 3.5;      // اختلاف با UTC (ایران +3.5)

// ================== پایه‌های استپر موتور ==================
const int PIN_STEP_AZ = 4;
const int PIN_DIR_AZ  = 5;
const int PIN_STEP_EL = 6;
const int PIN_DIR_EL  = 7;

// نسبت پالس به درجه (قابل تنظیم بر اساس درایور و گیربکس)
const double STEPS_PER_DEGREE_AZ = 50.0;
const double STEPS_PER_DEGREE_EL = 50.0;

double currentAzimuth = 90.0;   // موقعیت پارک: رو به مشرق
double currentElevation = 10.0; // شیب امن در حالت پارک

#define DEG_TO_RAD (PI / 180.0)
#define RAD_TO_DEG (180.0 / PI)

// ================== الگوریتم استاندارد نجومی موقعیت خورشید (NOAA) ==================
void calculateSolarPosition(DateTime now, double &azimuth, double &elevation) {
    int year = now.year();
    int month = now.month();
    int day = now.day();
    double hour = now.hour() - TIMEZONE + (now.minute() / 60.0) + (now.second() / 3600.0);

    if (month <= 2) {
        year -= 1;
        month += 12;
    }
    int A = year / 100;
    int B = 2 - A + (A / 4);
    double JD = floor(365.25 * (year + 4716)) + floor(30.6001 * (month + 1)) + day + B - 1524.5 + (hour / 24.0);
    double T = (JD - 2451545.0) / 36525.0;

    double L0 = fmod(280.46646 + T * (36000.76983 + T * 0.0003032), 360.0);
    double M  = fmod(357.52911 + T * (35999.05029 - 0.0001537 * T), 360.0);
    double C = (1.914602 - T * (0.004817 + 0.000014 * T)) * sin(M * DEG_TO_RAD)
             + (0.019993 - 0.000101 * T) * sin(2 * M * DEG_TO_RAD)
             + 0.000289 * sin(3 * M * DEG_TO_RAD);

    double sunTrueLong = L0 + C;
    double sunApparentLong = sunTrueLong - 0.00569 - 0.00478 * sin((125.04 - 1934.136 * T) * DEG_TO_RAD);

    double eps0 = 23.0 + (26.0 + ((21.448 - T * (46.8150 + T * (0.00059 - T * 0.001813)))) / 60.0) / 60.0;
    double eps  = eps0 + 0.00256 * cos((125.04 - 1934.136 * T) * DEG_TO_RAD);

    double declination = asin(sin(eps * DEG_TO_RAD) * sin(sunApparentLong * DEG_TO_RAD)) * RAD_TO_DEG;
    double ra = atan2(cos(eps * DEG_TO_RAD) * sin(sunApparentLong * DEG_TO_RAD), cos(sunApparentLong * DEG_TO_RAD)) * RAD_TO_DEG;

    double GMST0 = fmod(280.46061837 + 360.98564736629 * (JD - 2451545.0) + T * T * 0.000387933, 360.0);
    double LocalSiderealTime = fmod(GMST0 + LONGITUDE, 360.0);
    double HourAngle = LocalSiderealTime - ra;
    if (HourAngle < -180.0) HourAngle += 360.0;
    if (HourAngle > 180.0)  HourAngle -= 360.0;

    double latRad = LATITUDE * DEG_TO_RAD;
    double decRad = declination * DEG_TO_RAD;
    double haRad  = HourAngle * DEG_TO_RAD;

    elevation = asin(sin(latRad) * sin(decRad) + cos(latRad) * cos(decRad) * cos(haRad)) * RAD_TO_DEG;
    azimuth = atan2(-sin(haRad), tan(decRad) * cos(latRad) - sin(latRad) * cos(haRad)) * RAD_TO_DEG;
    if (azimuth < 0) azimuth += 360.0;
}

void stepToAngle(int stepPin, int dirPin, double &currentAngle, double targetAngle, double stepsPerDeg) {
    double diff = targetAngle - currentAngle;
    if (fabs(diff) < 0.5) return; // بازه مرده برای جلوگیری از استهلاک

    int steps = abs((int)(diff * stepsPerDeg));
    digitalWrite(dirPin, (diff > 0) ? HIGH : LOW);

    for (int i = 0; i < steps; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(800);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(800);
    }
    currentAngle = targetAngle;
}

void setup() {
    Serial.begin(115200);
    pinMode(PIN_STEP_AZ, OUTPUT);
    pinMode(PIN_DIR_AZ, OUTPUT);
    pinMode(PIN_STEP_EL, OUTPUT);
    pinMode(PIN_DIR_EL, OUTPUT);

    if (!rtc.begin()) {
        Serial.println(F("خطا: ماژول DS3231 یافت نشد!"));
        while (1);
    }

    if (rtc.lostPower()) {
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    Serial.println(F("سیستم ترکر خورشیدی دو محوره قاین فعال شد."));
}

void loop() {
    DateTime now = rtc.now();
    double sunAzimuth = 0.0;
    double sunElevation = 0.0;

    calculateSolarPosition(now, sunAzimuth, sunElevation);

    Serial.print(F("زمان: "));
    Serial.print(now.hour()); Serial.print(':'); Serial.print(now.minute());
    Serial.print(F(" | سمْت: ")); Serial.print(sunAzimuth);
    Serial.print(F("° | ارتفاع: ")); Serial.print(sunElevation); Serial.println(F("°"));

    if (sunElevation > 0.0) {
        stepToAngle(PIN_STEP_AZ, PIN_DIR_AZ, currentAzimuth, sunAzimuth, STEPS_PER_DEGREE_AZ);
        stepToAngle(PIN_STEP_EL, PIN_DIR_EL, currentElevation, sunElevation, STEPS_PER_DEGREE_EL);
    } else {
        stepToAngle(PIN_STEP_AZ, PIN_DIR_AZ, currentAzimuth, 90.0, STEPS_PER_DEGREE_AZ);
        stepToAngle(PIN_STEP_EL, PIN_DIR_EL, currentElevation, 10.0, STEPS_PER_DEGREE_EL);
        Serial.println(F("شب: پنل در حالت پارک"));
    }

    delay(60000); // به‌روزرسانی هر ۱ دقیقه
}

