//ELEVEN THERMISTOR SCRIPT FOR ARDUINO MEGA

#define THERMISTORPIN0 A0
#define THERMISTORPIN1 A1
#define THERMISTORPIN2 A2
#define THERMISTORPIN3 A3
#define THERMISTORPIN4 A4
#define THERMISTORPIN5 A5
#define THERMISTORPIN6 A6
#define THERMISTORPIN7 A7
#define THERMISTORPIN8 A8
#define THERMISTORPIN9 A9
#define THERMISTORPIN10 A10
#define THERMISTORPIN11 A11
#define THERMISTORNOMINAL 10000  
#define TEMPERATURENOMINAL 25 
#define NUMSAMPLES 10
#define BCOEFFICIENTL 3895
#define BCOEFFICIENTA 3895
#define SERIESRESISTOR 10000 
unsigned long waqt;

uint16_t samplesa0[NUMSAMPLES];
uint16_t samplesa1[NUMSAMPLES];
uint16_t samplesa2[NUMSAMPLES];
uint16_t samplesa3[NUMSAMPLES];
uint16_t samplesa4[NUMSAMPLES];
uint16_t samplesa5[NUMSAMPLES];
uint16_t samplesa6[NUMSAMPLES];
uint16_t samplesa7[NUMSAMPLES];
uint16_t samplesa8[NUMSAMPLES];
uint16_t samplesa9[NUMSAMPLES];
uint16_t samplesa10[NUMSAMPLES];
uint16_t samplesa11[NUMSAMPLES];
 
void setup(void) {
  Serial.begin(9600);
  analogReference(EXTERNAL);
  pinMode(2, OUTPUT);
}

void loop(void) {
  uint8_t i;
  waqt = millis()/1000;
  digitalWrite(2, HIGH);
  delay(1800);
   for (i=0; i< NUMSAMPLES; i++) {          // take N samples in a row, with a slight delay
   samplesa0[i] = analogRead(THERMISTORPIN0);
   samplesa1[i] = analogRead(THERMISTORPIN1);
   samplesa2[i] = analogRead(THERMISTORPIN2);
   samplesa3[i] = analogRead(THERMISTORPIN3);
   samplesa4[i] = analogRead(THERMISTORPIN4);
   samplesa5[i] = analogRead(THERMISTORPIN5);
   samplesa6[i] = analogRead(THERMISTORPIN6);
   samplesa7[i] = analogRead(THERMISTORPIN7);
   samplesa8[i] = analogRead(THERMISTORPIN8);
   samplesa9[i] = analogRead(THERMISTORPIN9);
   samplesa10[i] = analogRead(THERMISTORPIN10);
   samplesa11[i] = analogRead(THERMISTORPIN11);
   delay(200);

  }
  
  digitalWrite(2, LOW);
  float avga0;
  float avga1;
  float avga2;
  float avga3;
  float avga4;
  float avga5;
  float avga6;
  float avga7;
  float avga8;
  float avga9;
  float avga10;
  float avga11;
  avga0 = 0;    // average all the samples out
  avga1 = 0;
  avga2 = 0;
  avga3 = 0;
  avga4 = 0;
  avga5 = 0;
  avga6 = 0;
  avga7 = 0;
  avga8 = 0;
  avga9 = 0;
  avga10 = 0;
  avga11 = 0;
  for (i=0; i< NUMSAMPLES; i++) {
     avga0 += samplesa0[i];
     avga1 += samplesa1[i];
     avga2 += samplesa2[i];
     avga3 += samplesa3[i];
     avga4 += samplesa4[i];
     avga5 += samplesa5[i];
     avga6 += samplesa6[i];
     avga7 += samplesa7[i];
     avga8 += samplesa8[i];
     avga9 += samplesa9[i];
     avga10 += samplesa10[i];
     avga11 += samplesa11[i];
  }

  avga0 = SERIESRESISTOR / (1023 /(avga0/NUMSAMPLES) - 1);
  avga1 = SERIESRESISTOR / (1023 /(avga1/NUMSAMPLES) - 1);
  avga2 = SERIESRESISTOR / (1023 /(avga2/NUMSAMPLES) - 1);
  avga3 = SERIESRESISTOR / (1023 /(avga3/NUMSAMPLES) - 1);
  avga4 = SERIESRESISTOR / (1023 /(avga4/NUMSAMPLES) - 1);
  avga5 = SERIESRESISTOR / (1023 /(avga5/NUMSAMPLES) - 1);
  avga6 = SERIESRESISTOR / (1023 /(avga6/NUMSAMPLES) - 1);
  avga7 = SERIESRESISTOR / (1023 /(avga7/NUMSAMPLES) - 1);
  avga8 = SERIESRESISTOR / (1023 /(avga8/NUMSAMPLES) - 1);
  avga9 = SERIESRESISTOR / (1023 /(avga9/NUMSAMPLES) - 1);
  avga10 = SERIESRESISTOR / (1023 /(avga10/NUMSAMPLES) - 1);
  avga11 = SERIESRESISTOR / (1023 /(avga11/NUMSAMPLES) - 1);

  float steinharta0;
  float steinharta1;
  float steinharta2;
  float steinharta3;
  float steinharta4;
  float steinharta5;
  float steinharta6;
  float steinharta7;
  float steinharta8;
  float steinharta9;
  float steinharta10;
  float steinharta11;
  steinharta0 = 1/((log(avga0 / THERMISTORNOMINAL))/BCOEFFICIENTL + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;     // (R/Ro)
  steinharta1 = 1/((log(avga1 / THERMISTORNOMINAL))/BCOEFFICIENTA + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
  steinharta2 = 1/((log(avga2 / THERMISTORNOMINAL))/BCOEFFICIENTL + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
  steinharta3 = 1/((log(avga3 / THERMISTORNOMINAL))/BCOEFFICIENTA + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
  steinharta4 = 1/((log(avga4 / THERMISTORNOMINAL))/BCOEFFICIENTL + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
  steinharta5 = 1/((log(avga5 / THERMISTORNOMINAL))/BCOEFFICIENTA + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
  steinharta6 = 1/((log(avga6 / THERMISTORNOMINAL))/BCOEFFICIENTL + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
  steinharta7 = 1/((log(avga7 / THERMISTORNOMINAL))/BCOEFFICIENTA + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
  steinharta8 = 1/((log(avga8 / THERMISTORNOMINAL))/BCOEFFICIENTL + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
  steinharta9 = 1/((log(avga9 / THERMISTORNOMINAL))/BCOEFFICIENTA + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
  steinharta10 = 1/((log(avga10 / THERMISTORNOMINAL))/BCOEFFICIENTL + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
  steinharta11 = 1/((log(avga11 / THERMISTORNOMINAL))/BCOEFFICIENTL + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
  
  Serial.print(waqt);
  Serial.print(" A0 ");
  Serial.println(steinharta0);
  Serial.print(waqt);
  Serial.print(" A1 ");
  Serial.println(steinharta1);
  Serial.print(waqt);
  Serial.print(" A2 ");
  Serial.println(steinharta2);
  Serial.print(waqt);
  Serial.print(" A3 ");
  Serial.println(steinharta3);
  Serial.print(waqt);
  Serial.print(" A4 ");
  Serial.println(steinharta4);
  Serial.print(waqt);
  Serial.print(" A5 ");
  Serial.println(steinharta5);
  Serial.print(waqt);
  Serial.print(" A6 ");
  Serial.println(steinharta6);
  Serial.print(waqt);
  Serial.print(" A7 ");
  Serial.println(steinharta7);
  Serial.print(waqt);
  Serial.print(" A8 ");
  Serial.println(steinharta8);
  Serial.print(waqt);
  Serial.print(" A9 ");
  Serial.println(steinharta9);
  Serial.print(waqt);
  Serial.print(" A10 ");
  Serial.println(steinharta10);
  Serial.print(waqt);
  Serial.print(" A11 ");
  Serial.println(steinharta11);
  delay(3000);
}

  //average /= NUMSAMPLES;
  //average = 1023 / average - 1;           // convert the value to resistance
  //average = SERIESRESISTOR / average;

  //steinhart = log(steinhart);                  // ln(R/Ro)
  //steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  //steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  //steinhart = 1.0 / steinhart;                 // Invert
  //steinhart -= 273.15;                         // convert to C
