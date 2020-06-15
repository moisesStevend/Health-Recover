TaskHandle_t Task1;
TaskHandle_t Task2;

#include <ArduinoJson.h>
#include <SparkFunMPU9250-DMP.h>
#include "WiFi.h"
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define showBluetooth
//#define no_showBluetooth

#define SerialPort Serial
BluetoothSerial SerialBT;

MPU9250_DMP imu;

unsigned long stepCount = 0;
unsigned long stepTime = 0;
unsigned long lastStepCount = 0;

const signed char orientationMatrix[9] = {
  1, 0, 0,
  0, 1, 0,
  0, 0, 1
};

unsigned char lastOrient = 0;
boolean enabled=false;
int move=0;
char dat[2000];
float percent_c=0;
int rep_c=0;
float total_r=10;

//move 2
int count_enable=0;
int count=0;

/*************Generals variables pins config****************/
int battPin=34;
int led_indicator=12;
int size_gpio_leds=6;
int gpio_leds[6]={16,4,19,18,2,17};
int button=23;
uint8_t sensorId;
//variables for external interruption
bool fpress=true;

/*****move5 variables******/
int32_t accum=0;
int ac_enable=false;
int contador=0;

/******enable settings****/
int set_en_m1=true;
int set_en_m2=false;

/*******json requests**************/
String exe_accu="";

/****definition of functions***********/
void confGPIO(int *pines, int size_i, boolean test=false);
float readBattery(int pin);
void clear_led(int *pin);
void plot_led(int *pin, byte val);
void led_plot_Battery();
void send_BTMSG(char *msg);
void move1(void);
void move5(void);
void move2(void);
void move3(void);
void move4(void);
void move6(void);
void move7(void);
void move8(void);
void move9(void);
void move_test(void);

/***********External interruption***********/
void IRAM_ATTR isr(){
  //yield();
  delay(10);
  if(fpress){
    led_plot_Battery(gpio_leds);

  }else{
    clear_led(gpio_leds);
  }
  fpress = not(fpress);
}
/***********************************************/

void setup()
{
  SerialPort.begin(115200);
  //config button-indictor leds
  pinMode(button, INPUT_PULLUP);//config for the button
  pinMode(led_indicator,OUTPUT);
  digitalWrite(led_indicator, 1);
  attachInterrupt(button, isr, CHANGE);

  //config gpio-indictor leds
  confGPIO(gpio_leds, size_gpio_leds);//the config of the leds
  //led_plot_Battery(gpio_leds);

  SerialBT.begin("HealthRecover "+WiFi.macAddress().substring(12));
  Serial.println("The device started, now you can pair it with bluetooth!");
  send_BTMSG("Ingrese ejercicio: ");
  digitalWrite(led_indicator, 0);
  // Call imu.begin() to verify communication and initialize

  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }
  Serial.println("HR orientation ready2!!");

  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  imu.dmpBegin(DMP_FEATURE_ANDROID_ORIENT);
  imu.dmpSetOrientation(orientationMatrix);
  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(
    TaskBluetooth
    ,  "TaskBluetooth"   // A name just for humans
    ,  100000  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &Task1
    ,  0);

  xTaskCreatePinnedToCore(
    TaskGesture
    ,  "TaskGesture"
    ,  100000  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  &Task2
    ,  1);

}

void loop()
{
  if(enabled==true){
    switch(move){
      case 1:
        if(set_en_m2==false){
          imu.begin();
          imu.setSensors(INV_XYZ_GYRO); // Enable gyroscope only
          imu.setGyroFSR(2000); // Set gyro to 2000 dps

          imu.dmpBegin(DMP_FEATURE_GYRO_CAL |   // Enable gyro cal
              DMP_FEATURE_SEND_CAL_GYRO,// Send cal'd gyro values
              100);
              set_en_m1=false;
              set_en_m2=true;
        }
        move1();                // Set DMP rate to 10 Hz
        break;
      case 2:
        if(set_en_m1==false){
          imu.begin();
          imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
          imu.dmpBegin(DMP_FEATURE_ANDROID_ORIENT);
          imu.dmpSetOrientation(orientationMatrix);

          set_en_m1=true;
          set_en_m2=false;
        }

        move2();//listo
        break;
      case 3:
        if(set_en_m1==false){
          imu.begin();
          imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
          imu.dmpBegin(DMP_FEATURE_ANDROID_ORIENT);
          imu.dmpSetOrientation(orientationMatrix);
          set_en_m1=true;
          set_en_m2=false;
        }
        move3();
        break;
      case 4:
      if(set_en_m1==false){
        imu.begin();
        imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
        imu.dmpBegin(DMP_FEATURE_ANDROID_ORIENT);
        imu.dmpSetOrientation(orientationMatrix);
        set_en_m1=true;
        set_en_m2=false;
      }
        move4();
        break;
      case 5:
        if(set_en_m2==false){
          imu.begin();
          imu.setSensors(INV_XYZ_GYRO); // Enable gyroscope only
          imu.setGyroFSR(2000); // Set gyro to 2000 dps

          imu.dmpBegin(DMP_FEATURE_GYRO_CAL |   // Enable gyro cal
              DMP_FEATURE_SEND_CAL_GYRO,// Send cal'd gyro values
              100);                   // Set DMP rate to 10 Hz

              set_en_m1=false;
              set_en_m2=true;
        }

        move5();
        break;
      case 6:
        if(set_en_m1==false){
          imu.begin();
          imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
          imu.dmpBegin(DMP_FEATURE_ANDROID_ORIENT);
          imu.dmpSetOrientation(orientationMatrix);
          set_en_m1=true;
          set_en_m2=false;
        }
        move6();
        break;
      case 7:
        if(set_en_m1==false){
          imu.begin();
          imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
          imu.dmpBegin(DMP_FEATURE_ANDROID_ORIENT);
          imu.dmpSetOrientation(orientationMatrix);
          set_en_m1=true;
          set_en_m2=false;
        }
        move7();
        break;
      case 8:
        if(set_en_m1==false){
          imu.begin();
          imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
          imu.dmpBegin(DMP_FEATURE_ANDROID_ORIENT);
          imu.dmpSetOrientation(orientationMatrix);
          set_en_m1=true;
          set_en_m2=false;
        }
        move8();
        break;
      case 9:
        if(set_en_m1==false){
          imu.begin();
          imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
          imu.dmpBegin(DMP_FEATURE_ANDROID_ORIENT);
          imu.dmpSetOrientation(orientationMatrix);
          set_en_m1=true;
          set_en_m2=false;
        }
        move9();
        break;
    }
  }else{


  }
}

void TaskBluetooth(void *pvParameters)  // This is a task.
{
  while(1){
    delay(10);
    if (SerialBT.available()) {
      char exe = SerialBT.read();
      exe_accu+=exe;

      //String input = "{\"status\":\"play\",\"type\":\"exercise_01\"}";
      //{"status":"play", "type": "exercise_01"}
      if(exe=='\n'){
      //if(1==1){
        Serial.println(exe_accu);
        //Serial.println(input);
        DynamicJsonDocument root(2048);
        //deserializeJson(root, input);
        deserializeJson(root, exe_accu);
        JsonObject obj = root.as<JsonObject>();

        String exe_status = obj[String("status")];
        int exe_type = obj[String("exe")];

        //snprintf(dat,2000,"status: %s\n,type: %d", exe_status, exe_type);
        //send_BTMSG(dat);

        if(exe_status=="play"){

          int exe_i = exe_type;//(int)exe_type - 48;
          if(0<=exe_i && exe_i<=9){
            move = exe_i;
            enabled=true;
            Serial.println("exercise: "+String(move));
            Serial.println("enabled: "+String(enabled));
            send_BTMSG("{'status': 'ok-play'}");
          }else{
            send_BTMSG("{'status': 'failed'}");
          }

        }else if(exe_status=="stop"){
          enabled=false;
          send_BTMSG("{'status': 'ok-stop'}");
          //send_BTMSG("\nIngrese ejercicio: ");
        }else if(exe_status=="get"){
          //send_BTMSG("{'status': 'ok-get'}");
          enabled=false;
          rep_c=count;
          percent_c=(rep_c/total_r)*100;
          snprintf(dat,2000,"{'status': 'ok-get', 'correctos': %d ,'porcentaje': %.2f, 'ejercicio': %d  }",rep_c, percent_c, move);
          send_BTMSG(dat);
          count=0;
        }

        //snprintf(dat,2000,"status: %s\n,type: %s, \nok: %s", exe_status, exe_type, "ok");
        //send_BTMSG(dat);
        exe_accu="";
      }

/*
      if(exe=='y'){
        send_BTMSG("Inicie ejercicio >_< ");
        enabled=true;
        Serial.println(enabled);
      }else if(exe=='n'){
        enabled=false;
        send_BTMSG("\nIngrese ejercicio: ");
      }else if(exe=='c'){
        enabled=false;
        rep_c=count;
        percent_c=(rep_c/total_r)*100;
        snprintf(dat,500,"Correctos: %d \nPorcentaje: %.2f",rep_c, percent_c);
        send_BTMSG(dat);
        count=0;
      }
      else {
        int exe_i = (int)exe - 48;
        if(0<=exe_i && exe_i<=9){
          enabled=false;
          move = exe_i;
          snprintf(dat,500,"Confirmar ejercicio %d (y/n):  ",move);
          send_BTMSG(dat);

        }
      }
      */
    }
  }
}

void TaskGesture(void *pvParameters)  // This is a task.
{


  for(;;){
     //Serial.println(digitalRead(33));
     delay(100);


   }
}

#ifdef showBluetooth
void send_BTMSG(char *msg){
  SerialBT.write('\n');
  for(int i=0;i<strlen(msg);i++){
    SerialBT.write(msg[i]);
  }
}
#endif

#ifdef no_showBluetooth
void send_BTMSG(char *msg){

}
#endif


/**************function button*************************/
void confGPIO(int *pines, int size_i, boolean test){
  for(int i=0;i<size_i;i++){
    pinMode(pines[i],OUTPUT);
  }

  if(test){
    boolean change=true;
    int rep=4;

    //first for
    for(int j=0;j<rep; j++){
        for (int i=0;i<size_i;i++){
          digitalWrite(pines[i],change);
        }
        delay(500);
        change=not(change);

        for (int i=0;i<size_i;i++){
          digitalWrite(pines[i],change);
        }
        delay(500);
        change=not(change);
    }

    //second for
    for(int j=0;j<rep; j++){
        for (int i=0;i<size_i;i++){
          digitalWrite(pines[i],1);
          delay(500);
          digitalWrite(pines[i],0);
        }
    }

  }
}

/*************functions of movement*********/

void move1(void){
  if ( imu.fifoAvailable() )
  {
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {

      float gyroX = imu.calcGyro(imu.gx);
      float gyroY = imu.calcGyro(imu.gy);
      float gyroZ = imu.calcGyro(imu.gz);

      SerialPort.println(String(gyroZ) + " dps");
      SerialPort.println("Time: " + String(imu.time) + " ms");
      SerialPort.println("count++: "+String(count));
      Serial.println();

      if(-2<=gyroZ && gyroZ<=1){
        ac_enable=true;
        accum=0;
      }else{
        //ac_enable=false;
        if(gyroZ>1){
          ac_enable=false;
        }
      }

      if(gyroZ<=-1 && ac_enable==true){
        accum=0;

        while(gyroZ<-5){
            imu.dmpUpdateFifo();
            gyroZ = imu.calcGyro(imu.gz);
            SerialPort.println("B:"+String(gyroZ) + " dps");
            SerialPort.println("b: Time: " + String(imu.time) + " ms");
            SerialPort.println();

            accum+=gyroZ;

            if(gyroZ>0)
              break;
        }
      }


      SerialPort.println("enable: "+String(ac_enable));

      if(1<=gyroZ && ac_enable==false){
        accum=0;

        while(5<gyroZ){
            imu.dmpUpdateFifo();
            gyroZ = imu.calcGyro(imu.gz);
            SerialPort.println("B:"+String(gyroZ) + " dps");
            SerialPort.println("b: Time: " + String(imu.time) + " ms");
            SerialPort.println();

            accum+=gyroZ;
            if(gyroZ<0)
              break;
        }
        Serial.println("grades: "+String(accum));
        if(accum<-20000){
          if(contador==2){
             count++;
            }
            contador=1;
          }
      }



        Serial.println("grades: "+String(accum));
         if(accum>20000 && contador==1){
            contador=2;
          }
      }
    }
}


void move5(void){
  if ( imu.fifoAvailable() )
  {
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {

      float gyroX = imu.calcGyro(imu.gx);
      float gyroY = imu.calcGyro(imu.gy);
      float gyroZ = imu.calcGyro(imu.gz);

      SerialPort.println(String(gyroZ) + " dps");
      SerialPort.println("Time: " + String(imu.time) + " ms");
      SerialPort.println("count++: "+String(count));
      Serial.println();

      if(-1<=gyroZ && gyroZ<=1){
        ac_enable=true;
        accum=0;
      }else{
        //ac_enable=false;
        if(gyroZ<-1){
          ac_enable=false;
        }
      }

      SerialPort.println("enable: "+String(ac_enable));

      if(1<=gyroZ && ac_enable==true){
        accum=0;

        while(5<gyroZ){
            imu.dmpUpdateFifo();
            gyroZ = imu.calcGyro(imu.gz);
            SerialPort.println("B:"+String(gyroZ) + " dps");
            SerialPort.println("b: Time: " + String(imu.time) + " ms");
            SerialPort.println();

            accum+=gyroZ;
            if(gyroZ<0)
              break;
        }
        Serial.println("grades: "+String(accum));
        if(accum>50000){
          if(contador==2){
             count++;
            }
            contador=1;
          }
      }


      if(gyroZ<=-1 && ac_enable==false){
        accum=0;

        while(gyroZ<-5){
            imu.dmpUpdateFifo();
            gyroZ = imu.calcGyro(imu.gz);
            SerialPort.println("B:"+String(gyroZ) + " dps");
            SerialPort.println("b: Time: " + String(imu.time) + " ms");
            SerialPort.println();

            accum+=gyroZ;

            if(gyroZ>0)
              break;
        }
        }
        Serial.println("grades: "+String(accum));
         if(accum<-40000 && contador==1){
            contador=2;
          }
      }
    }
}


void move2(){
  int seq[2]={ORIENT_PORTRAIT,ORIENT_REVERSE_LANDSCAPE};

  if ( imu.fifoAvailable() )
  {
    imu.dmpUpdateFifo();
    unsigned char orient = imu.dmpGetOrientation();
    //if(orient==ORIENT_PORTRAIT)

    if (orient != lastOrient)
    {
      switch (orient)
      {
      case ORIENT_PORTRAIT:
        if(count_enable==2){
          count++;
        }
        count_enable=1;

        SerialPort.println("Portrait");
        //send_BTMSG("Portrait");
        break;
      case ORIENT_LANDSCAPE:
        count_enable=0;
        SerialPort.println("Landscape");
        //send_BTMSG("Landscape");
        break;
      case ORIENT_REVERSE_PORTRAIT:
        count_enable=0;
        SerialPort.println("Portrait (Reverse)");
        //send_BTMSG("Portrait (Reverse)");
        break;
      case ORIENT_REVERSE_LANDSCAPE:
        if(count_enable==1){
          count_enable=2;
        }
        SerialPort.println("Landscape (Reverse)");
        //send_BTMSG("Landscape (Reverse)");
        break;
      }
      lastOrient = orient;
    }
  }

}


void move3(){
  //LandscapePortrait
  if ( imu.fifoAvailable() )
  {
    imu.dmpUpdateFifo();
    unsigned char orient = imu.dmpGetOrientation();
    //if(orient==ORIENT_PORTRAIT)

    if (orient != lastOrient)
    {
      switch (orient)
      {
      case ORIENT_PORTRAIT:
        if(count_enable==2){
          count++;
        }
        count_enable=1;

        SerialPort.println("Portrait");
        //send_BTMSG("Portrait");
        break;
      case ORIENT_LANDSCAPE:
        if(count_enable==1){
          count_enable=2;
        }
        SerialPort.println("Landscape");
        //send_BTMSG("Landscape");
        break;
      case ORIENT_REVERSE_PORTRAIT:
        count_enable=0;
        SerialPort.println("Portrait (Reverse)");
        //send_BTMSG("Portrait (Reverse)");
        break;
      case ORIENT_REVERSE_LANDSCAPE:
        count_enable=0;
        SerialPort.println("Landscape (Reverse)");
        //send_BTMSG("Landscape (Reverse)");
        break;
      }
      lastOrient = orient;
    }
  }
}

void move4(){
  //Landscape (Reverse)Portrait
  if ( imu.fifoAvailable() )
  {
    imu.dmpUpdateFifo();
    unsigned char orient = imu.dmpGetOrientation();

    if (orient != lastOrient)
    {
      switch (orient)
      {
      case ORIENT_PORTRAIT:
        if(count_enable==1){
          count_enable=2;
        }
        SerialPort.println("Portrait");
        //send_BTMSG("Portrait");
        break;
      case ORIENT_LANDSCAPE:
        count_enable=0;
        SerialPort.println("Landscape");
        //send_BTMSG("Landscape");
        break;
      case ORIENT_REVERSE_PORTRAIT:
        count_enable=0;
        SerialPort.println("Portrait (Reverse)");
        //send_BTMSG("Portrait (Reverse)");
        break;
      case ORIENT_REVERSE_LANDSCAPE:
        if(count_enable==2){
          count++;
        }
        count_enable=1;
        SerialPort.println("Landscape (Reverse)");
        //send_BTMSG("Landscape (Reverse)");
        break;
      }
      lastOrient = orient;
    }
  }
}

void move6(){
  //Landscape (Reverse)Landscape
  if ( imu.fifoAvailable() )
  {
    imu.dmpUpdateFifo();
    unsigned char orient = imu.dmpGetOrientation();

    if (orient != lastOrient)
    {
      switch (orient)
      {
      case ORIENT_PORTRAIT:
        count_enable=0;
        SerialPort.println("Portrait");
        //send_BTMSG("Portrait");
        break;
      case ORIENT_LANDSCAPE:
        if(count_enable==1){
          count_enable=2;
        }

        SerialPort.println("Landscape");
        //send_BTMSG("Landscape");
        break;
      case ORIENT_REVERSE_PORTRAIT:
        count_enable=0;
        SerialPort.println("Portrait (Reverse)");
        //send_BTMSG("Portrait (Reverse)");
        break;
      case ORIENT_REVERSE_LANDSCAPE:
        if(count_enable==2){
          count++;
        }
        count_enable=1;
        SerialPort.println("Landscape (Reverse)");
        //send_BTMSG("Landscape (Reverse)");
        break;
      }
      lastOrient = orient;
    }
  }
}

void move7(){
  //PortraitLandscape
  if ( imu.fifoAvailable() )
  {
    imu.dmpUpdateFifo();
    unsigned char orient = imu.dmpGetOrientation();

    if (orient != lastOrient)
    {
      switch (orient)
      {
      case ORIENT_PORTRAIT:
        if(count_enable==2){
          count++;
        }
        count_enable=1;

        SerialPort.println("Portrait");
        //send_BTMSG("Portrait");
        break;
      case ORIENT_LANDSCAPE:
        if(count_enable==1){
          count_enable=2;
        }

        SerialPort.println("Landscape");
        //send_BTMSG("Landscape");
        break;
      case ORIENT_REVERSE_PORTRAIT:
        count_enable=0;
        SerialPort.println("Portrait (Reverse)");
        //send_BTMSG("Portrait (Reverse)");
        break;
      case ORIENT_REVERSE_LANDSCAPE:
        count_enable=0;
        SerialPort.println("Landscape (Reverse)");
        //send_BTMSG("Landscape (Reverse)");
        break;
      }
      lastOrient = orient;
    }
  }
}
//falta
void move8(){
  //Landscape (Reverse)Portrait
  if ( imu.fifoAvailable() )
  {
    imu.dmpUpdateFifo();
    unsigned char orient = imu.dmpGetOrientation();

    if (orient != lastOrient)
    {
      switch (orient)
      {
      case ORIENT_PORTRAIT:
        if(count_enable==1){
          count_enable=2;
        }
        SerialPort.println("Portrait");
        //send_BTMSG("Portrait");
        break;
      case ORIENT_LANDSCAPE:
        count_enable=0;
        SerialPort.println("Landscape");
        //send_BTMSG("Landscape");
        break;
      case ORIENT_REVERSE_PORTRAIT:
        count_enable=0;
        SerialPort.println("Portrait (Reverse)");
        //send_BTMSG("Portrait (Reverse)");
        break;
      case ORIENT_REVERSE_LANDSCAPE:
        if(count_enable==2){
          count++;
        }
        count_enable=1;
        SerialPort.println("Landscape (Reverse)");
        //send_BTMSG("Landscape (Reverse)");
        break;
      }
      lastOrient = orient;
    }
  }
}

void move9(){
  //PortraitLandscape
  if ( imu.fifoAvailable() )
  {
    imu.dmpUpdateFifo();
    unsigned char orient = imu.dmpGetOrientation();

    if (orient != lastOrient)
    {
      switch (orient)
      {
      case ORIENT_PORTRAIT:
        if(count_enable==2){
          count++;
        }
        count_enable=1;

        SerialPort.println("Portrait");
        //send_BTMSG("Portrait");
        break;
      case ORIENT_LANDSCAPE:
        if(count_enable==1){
          count_enable=2;
        }

        SerialPort.println("Landscape");
        //send_BTMSG("Landscape");
        break;
      case ORIENT_REVERSE_PORTRAIT:
        count_enable=0;
        SerialPort.println("Portrait (Reverse)");
        //send_BTMSG("Portrait (Reverse)");
        break;
      case ORIENT_REVERSE_LANDSCAPE:
        count_enable=0;
        SerialPort.println("Landscape (Reverse)");
        //send_BTMSG("Landscape (Reverse)");
        break;
      }
      lastOrient = orient;
    }
  }
}

void move_test(){
  if ( imu.fifoAvailable() )
  {
    imu.dmpUpdateFifo();
    unsigned char orient = imu.dmpGetOrientation();

    if (orient != lastOrient)
    {
      switch (orient)
      {
      case ORIENT_PORTRAIT:


        SerialPort.println("Portrait");
        send_BTMSG("Portrait");
        break;
      case ORIENT_LANDSCAPE:

        SerialPort.println("Landscape");
        send_BTMSG("Landscape");
        break;
      case ORIENT_REVERSE_PORTRAIT:

        SerialPort.println("Portrait (Reverse)");
        send_BTMSG("Portrait (Reverse)");
        break;
      case ORIENT_REVERSE_LANDSCAPE:

        SerialPort.println("Landscape (Reverse)");
        send_BTMSG("Landscape (Reverse)");
        break;
      }
      lastOrient = orient;
    }
  }
}

/*************led-button functions*********/
float readBattery(int pin){
  float bat=0;
  bat = analogRead(pin)*3.3/4096.0;//0-4096

  return bat;
}

void clear_led(int *pin){
  //val = 0b001110;
  for(int i=0;i<size_gpio_leds;i++){
    digitalWrite(pin[i],0);
  }
}

void plot_led(int *pin, byte val){
  //val = 0b001110;
  clear_led(pin);

  for(int i=0;i<size_gpio_leds;i++){
    digitalWrite(pin[i],(val&(0b000001<<i))>>i);
  }
}

void led_plot_Battery(int *pin){
  float lectbat = readBattery(battPin);
  if(lectbat > 3.2){
    //111111
    plot_led(pin, 0b111111);
    //plot_led(pin, 0b001111);
    //plot_led(pin, 0b000111);
  }else if(3.1<lectbat && lectbat<=3.2) {
    //001111
    plot_led(pin, 0b001111);
  }else if(2.6<lectbat && lectbat<=3.1){
    //000111
    plot_led(pin, 0b000111);
  }else{
    //000001
    plot_led(pin, 0b000001);
  }
}
