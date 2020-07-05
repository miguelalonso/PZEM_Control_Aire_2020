
// definimos el lugar, sun y time de acuerdo con sunpos.h
// cLocation lugar;
// cSunCoordinates sun;
// cTime fecha_hora_PSA;

 spa_data spa;
//************************************************//
//*******************PZEM-016*********************//
//************************************************//
#define MAX485_DE      D1
#define MAX485_RE_NEG  D2

struct datos_pzem016 {
      double voltage;
      double current ;
      double power;
      double energy ;
      double frequency;
      double power_factor;
} pzem;

struct datos_tasmotas{
  float Total;
  float Today;;
  float Yesterday;
  float Power;
  float ApparentPower;
  float ReactivePower;
  float Factor;
  float Voltage;
  float Current;
  int   estado;
 
} inversor, aire;


String payload_inversor;
String payload_aire;
 
      
//#define ssid      "MiFibra-F870"      // WiFi SSID
//#define password  "P9EnMuxD"  // WiFi password
//#define DHTTYPE   DHT22           // DHT type (DHT11, DHT22)
//#define DHTPIN    D0              // Broche du DHT / DHT Pin
const uint8_t GPIOPIN[6] = {D0,D3,D7,D8};  // cambiar D4 a D5
#define      Relay_D0        D0   // (D0) 
#define      Relay_D3        D3   // (D6) 
#define      Relay_D7        D7   // (D7) 
#define      Relay_D8        D8   // (D8) 


//float     t = 0 ;
//float     h = 0 ;
//float     pa = 0;
boolean     aire_estado_anterior;
boolean     inversor_estado_anterior;
boolean   estados[6]; //estados de los relés //se añaden los D0 y D4 calentador y ventilador
int       Num_reles=4;
int       i,j,k;
char      username[] = "074fdc60-977a-11ea-93bf-d33a96695544"; //Cayenne
char      password[] = "467869b0865fbac3b961d4a02907a972b15f92d6";
char      clientID[] = "e65003d0-bab2-11ea-a67f-15e30d90bbf4";
long      time_old[4];

struct StoreStruct {
  int           Mode;                       
  boolean       Estado_rele;
  char          ReleName[30];     
  int           ON_Time[7];     //tiempo de cada programa, puede ser para ON u OFF en función de "onoff"
  int           OFF_Time[7];    //ya no se utiliza, reserva
  boolean       ON_Days[7][8];
  char          progName[7][30];
  int           sun_set[7];       //0:timer, 1: sunrise, 2:sunset
  int           sign_delay[7];    //0 es +; 1 es -
  int           onoff[7];         // en caso de sunrise o sunset pone en on(1) o en off(0)
  int           delay_time[7];    // variable de tiempo de retraso/adelanto para sunrise y sunset.
  int           aleatorio_Time[7]; // variable de tiempo aleatorio.
  int           Tiempo_procesado[7]; //tiempos aleatorios procesados para control
 } timers[4] ;

struct StoreStruct2 {
  char          version[4];
  int           EEPROM_chk;        // EEPROM Check
  char          DevName[30];       // Device name, i.e Sistema 3 - Aireador
  float         Latitud;
  float         Longitud;
  int           TimeZone;
  int           IP_1;
  int           IP_2;
  int           IP_3;
  int           IP_4;
  char          username[50] ;
  char          password[50] ;
  char          clientID[50];
  char          pwd[8];
  int           sunrise;
  int           sunset;
  float         temp_max;
  float         temp_min;
  int           temp_mode;
  char          ip_inversor[16];
  char          ip_aire[16];
} conf ;



unsigned long lastMillis = 0;

// Variables
//----------
#define      ErrMsg            "<font color=\"red\"> < < Error !</font>"
boolean      ResetWiFi       = false;
long         timeNow         = 0;
long         timeOld         = 300000;
long         timeOld2         = 0;
long         timeOld3         = 0;
long         timeOld4         = 0;
long         timeOld5         = 0;
boolean      TimeOk          = false;
boolean      NTPtimeOk       = false;
String       request         = "";
byte         Page            = 1;
boolean      TimeCheck       = false;
int          NewHH           = 0;
int          NewMM           = 0;
int          Newdd           = 0;
int          Newmm           = 0;
int          Newyy           = 0;
int          PgmNr           = 0;
int          LastHH          = 0;
int          LastMM          = 0;
int          Lastdd          = 0;
int          Lastmm          = 0;
int          Lastyy          = 0;
byte         old_sec         = 0;
long         old_ms          = 0;
boolean      PgmPrev         = false;
boolean      PgmNext         = false;
boolean      PgmSave         = false;
boolean      Error1          = false;
boolean      Error2          = false;
boolean      Error3          = false;
boolean      Error3b          = false;
boolean      Error4          = false;
boolean      Error5          = false;
boolean      Error6          = false;
boolean      Error7          = false;
boolean      Error8          = false;
boolean      D[8]            = {false, false, false, false, false, false, false, false};
int          PgmNrG           = 0;
int          OnHour          = 0;
int          OnMinute        = 0;
int          OffHour         = 0;
int          OffMinute       = 0;


 
// Defines
#define      DefaultName       "PZEM016"  // Default device name
#define      NTPfastReq        10                 // NTP time request in seconds when  time not set
#define      NTPslowReq        3600               // NTP time request in seconds after time is  set
#define      Version           "1.00"             // Firmware version
String Argument_Name;

// NTP Server details
//-------------------
IPAddress timeServer(129, 6, 15, 28);              // time.nist.gov NTP server
WiFiUDP Udp;
unsigned int localPort       = 3232;               // Local port to listen for UDP packets
bool shouldSaveConfig = false;

ESP8266WebServer server ( 80 );
