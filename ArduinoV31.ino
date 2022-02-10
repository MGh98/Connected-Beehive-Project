//V30, lundi 3 janvier et mardi 4 janvier
//A faire version finale : mettre en commentaires tous les serial.println (pour affichage moniteur) (peut-être économiser quelques mA)


#include "HX711.h" // bibli poids
#include "DHT.h" // bibliothèque dht22
#include "Arduino_HTS221.h" // bibliothèque TempExtérieure intégrée Arduino NANO 

#include <Beees_inferencing.h> //V22
#include <PDM.h>
#define EIDSP_QUANTIZE_FILTERBANK   0

#define DHTPIN 5

#define LED_ALLUM 7

#define MAXIMWIRE_EXTERNAL_PULLUP 
// bibliotheque TempINt
#include <MaximWire.h> 
#define PIN_BUS 9
#define PIN_BUS2 10
#define PIN_BUS3 8

int A_Pin_photo = A1; 
// lire tension batterie
int A_Pin_3_7 = A0; 

MaximWire::Bus bus(PIN_BUS);
MaximWire::Bus bus2(PIN_BUS2);
MaximWire::Bus bus3(PIN_BUS3);
MaximWire::DS18B20 device;

#define DHTTYPE DHT22

const int LOADCELL_DOUT_PIN = 3;
const int LOADCELL_SCK_PIN = 4;

HX711 scale;

DHT dht(DHTPIN, DHTTYPE);


//--------------------------------------------Structures--------------------------------------------


typedef struct {
     // On multiplie par 2 pour envoyer un int qu'on divise par 2 sur Ubidots
     unsigned short HumiditeExterne : 8; 
     unsigned short HumiditeInterne : 8;  
     // 8 états, on stocke sur un octet (traitement arithmétique sur Ubidots)
     unsigned short Batterie : 8;
     //On utilise le capteur de la carte, on multiplie par 2 puis divise par 2 (précision 0.5°C)
     signed short TempExterne: 8; 
     // On utilise la sonde Grove en 0.5°c de précision
     signed short TempInt: 8; 
     signed short TempInt2: 8;
     signed short TempInt3: 8;
     //Poids*10 pour à précision de 100g (de 0 à 1500) 
     unsigned short Poids: 12; 
     // 8 états
     unsigned short Luminosite: 8; 
     //information sur 4 bits: Reine/SansReine/RucheActive/NonIdentifiable
     unsigned short Audio: 8; 
} Data;

typedef struct {
    //On envoie trois int de 32 bits contenant nos infos
    signed int Envoi: 32;
    signed int Envoi2: 32;
    unsigned int Envoi3: 32;
} BonEnvoi;


typedef struct {
    //float tempInt1;
    signed int tempInt1;
    //float tempInt2;
    signed int tempInt2;
    //float tempInt3;
    signed int tempInt3;
} threetemperatures;

typedef struct {
    int16_t *buffer;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

static inference_t inference;
static signed short sampleBuffer[2048];
// Set this to true to see e.g. features generated from the raw signal
static bool debug_nn = false; 


//--------------------------------------------Setup--------------------------------------------
void setup() {

    Serial1.begin(9600);
    Serial.begin(9600);
    
    digitalWrite(LED_PWR, LOW);
    //Eteindre capteur de poids très gourmand en consommation
    digitalWrite(4, HIGH); 
    delay(100);
  
    pinMode(LED_ALLUM, OUTPUT);
    digitalWrite(LED_ALLUM, HIGH);
    delay(4500);
    digitalWrite(LED_ALLUM, LOW);
    Serial.println("Hellooo\n");
  
    //Allumer capteur de poids
    digitalWrite(4, LOW); 
    delay(1000);
    //Pour poids
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    scale.set_offset(99000);
    //581733 mesuré pour 25.2 kg => 425500/15.7 = 27100
    scale.set_scale(30000.f);
  
    analogReadResolution(12);
    
    dht.begin();
  
    if (!HTS.begin()) {while (1);}
    
    if (microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT) == false) {return;}
}

//--------------------------------------------Fonctions--------------------------------------------

int fctNiveau_batterie() {
  int lecture_tension = analogRead(A_Pin_3_7);
  float tension_a_l_echelle = 3.3 * (float)lecture_tension / 4095.0;
  float tension_lineaire = tension_a_l_echelle - 2.70;
  int pourcentage_batterie = (tension_lineaire * 100) / 0.64;
  
  if (pourcentage_batterie >= 100) {pourcentage_batterie = 100;}
  else if (pourcentage_batterie <= 0) {pourcentage_batterie = 0;}
  
  int batterie_palier = pourcentage_batterie - pourcentage_batterie%10;
  if (batterie_palier == 0) {batterie_palier = 1;}

  //return paliers de 10% en 10%
  return batterie_palier; 
}

//Température externe = capteur temp intégré de la carte
//float fctTempExt() {
signed int fctTempExt() {
    float tempExt = HTS.readTemperature();
    signed int realTempExt = (signed int) tempExt * 2;
    // on multiplie par deux pour faire tenir les valeurs à 0.5° près en 8 bits
    //return tempExt*2; 
    return realTempExt;
}

//Humidité externe = pour l'instant le capteur interne à la carte
int fctHumidityExt() {
    int humidity = HTS.readHumidity();
    return humidity;
}

//Humidité externe = pour l'instant le DHT22
int fctHumidityInt() {
  int humidityInt = dht.readHumidity();
  if (isnan(humidityInt)) {return 0;}
  return humidityInt;
}

//2 capteurs de températures internes, sur la même pin / le même bus
//Capteur de température externe n°3, seul sur sa pin / bus
threetemperatures fctTempInt() {
  threetemperatures result;
  signed int temp1;
  signed int temp2;
  signed int temp3;

   MaximWire::Discovery discovery3 = bus3.Discover();
    do {
        MaximWire::Address address;
        if (discovery3.FindNextDevice(address)) {
            if (address.GetModelCode() == MaximWire::DS18B20::MODEL_CODE) {
              
              MaximWire::DS18B20 device(address);

                //numéro 1
              if (address.ToString() == "281A8670090000FB") {
                  //result.tempInt1 = device.GetTemperature<float>(bus3);
                  temp1 = device.GetTemperature<float>(bus3);
                  //result.tempInt1 = result.tempInt1 * 2;
                  temp1 = temp1 * 2;
                  device.Update(bus3);
                }
                
            } 
            //else {result.tempInt1 = 2.0;}
            else {temp1 = 2.0;}
        } 
    } while (discovery3.HaveMore());
    
    MaximWire::Discovery discovery = bus.Discover();
    do {
        MaximWire::Address address;
        if (discovery.FindNextDevice(address)) {
 
            if (address.GetModelCode() == MaximWire::DS18B20::MODEL_CODE) {
                
                MaximWire::DS18B20 device(address);
                  
                if (address.ToString() == "285193D50B0000BF"){
                  //result.tempInt2 = device.GetTemperature<float>(bus);
                  temp2 = device.GetTemperature<float>(bus);
                  //result.tempInt2 = result.tempInt2 * 2;
                  temp2 =  temp2 * 2;
                  device.Update(bus);
                }
            } 
            //else {result.tempInt2 = 2.0;}
            else {temp2 = 2.0;}
        } 
    } while (discovery.HaveMore());

    MaximWire::Discovery discovery2 = bus2.Discover();
    do {
        MaximWire::Address address;
        if (discovery2.FindNextDevice(address)) {

            if (address.GetModelCode() == MaximWire::DS18B20::MODEL_CODE) {
                
                MaximWire::DS18B20 device(address);

                //numéro 3

              if (address.ToString() == "28F194320C000007"){
                  //result.tempInt3 = device.GetTemperature<float>(bus2);
                  temp3 = device.GetTemperature<float>(bus2);
                  //result.tempInt3 = result.tempInt3 * 2;
                  temp3 = temp3 * 2;
                  device.Update(bus2);
                }
            } 
            //else {result.tempInt3 = 2.0;}
            else {temp3 = 2.0;}
        } 
    } while (discovery2.HaveMore());

    if (temp1 == 0.0) {temp1 = temp2;}
    result.tempInt1 = (signed int) temp1;
    result.tempInt2 = (signed int) temp2;
    result.tempInt3 = (signed int) temp3;
    
    return result;
}


int fctPoids() {
    digitalWrite(4, LOW); 
    //Allumer capteur de poids
    delay(1000); 
    //Attendre que le capteur s'allume....

    if (scale.is_ready()) {
      float reading = scale.get_units(1);
      int res = 10 * reading;
      
      digitalWrite(4, HIGH);   
      //Eteindre capteur de poids très gourmand en consommation pour réduire celle-ci
      delay(100);
      
      return res;
    } 
    
  else {
        digitalWrite(4, HIGH); 
        //Eteindre capteur de poids très gourmand en consommation pour réduire celle-ci
        delay(100);
        return 0;
    }
}

//ne marche pas....
int fctLuminosite(){
  int val_U_luminosite = analogRead(A_Pin_photo);
  
  float tension_U_luminosite = 3.3 * val_U_luminosite / 4095; 
  int finalres = (int) (100.0 / tension_U_luminosite);

  if (finalres - 30 >= 0) {finalres = finalres - 30;}
  else {finalres = finalres - 20;}
  return finalres;
}

static void pdm_data_ready_inference_callback(void) {
    int bytesAvailable = PDM.available();

    // read into the sample buffer
    int bytesRead = PDM.read((char *)&sampleBuffer[0], bytesAvailable);

    if (inference.buf_ready == 0) {
        for(int i = 0; i < bytesRead>>1; i++) {
            inference.buffer[inference.buf_count++] = sampleBuffer[i];

            if(inference.buf_count >= inference.n_samples) {
                inference.buf_count = 0;
                inference.buf_ready = 1;
                break;
            }
        }
    }
}


/* Init inferencing struct and setup/start PDM
 * return { description_of_the_return_value } */
static bool microphone_inference_start(uint32_t n_samples) {
    inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));

    if (inference.buffer == NULL) {return false;}

    inference.buf_count  = 0;
    inference.n_samples  = n_samples;
    inference.buf_ready  = 0;

    // configure the data receive callback
    PDM.onReceive(&pdm_data_ready_inference_callback);
    PDM.setBufferSize(4096);

    // initialize PDM with:
    // - one channel (mono mode)
    // - a 16 kHz sample rate
    if (!PDM.begin(1, EI_CLASSIFIER_FREQUENCY)) {
        ei_printf("Failed to start PDM!");
        microphone_inference_end();

        return false;
    }

    // set the gain, defaults to 20
    PDM.setGain(127);

    return true;
}

/* Wait on new data, return True when finished */
static bool microphone_inference_record(void) {
    inference.buf_ready = 0;
    inference.buf_count = 0;

    while (inference.buf_ready == 0) {delay(10);}

    return true;
}

/* Get raw audio signal data */
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
    numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);
    return 0;
}

/* Stop PDM and release buffers  */
static void microphone_inference_end(void) {
    PDM.end();
    free(inference.buffer);
}


#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif


//---------------------------//


//Fonction qui gere toute l'ia : enregistrer sons, run l'inference et print la pred la plus probable
unsigned int fctAudio() {
  unsigned int resultPredictions;
  int compt_normal = 0, compt_piping = 0, compt_voices = 0, compt_notbees = 0;
  
  for (size_t i = 0; i < 4; i++) {
      
      delay (500); //attendre 2 secondes entre chaque inférence
  
      bool m = microphone_inference_record();
      if (!m) {
            resultPredictions = 11;
            return resultPredictions;
      }
    
      signal_t signal;
      signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
      signal.get_data = &microphone_audio_signal_get_data;
      
      ei_impulse_result_t result = { 0 };
  
      EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
      if (r != EI_IMPULSE_OK) {
            resultPredictions = 12;
            return resultPredictions;
      }

      float maxi = 0;
      const char* Str = "";
      for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        if (result.classification[ix].value > maxi && result.classification[ix].value > 0.6) {
          maxi = result.classification[ix].value;
          Str = result.classification[ix].label;
        }
      }
  
        if (Str == "normal") {compt_normal++;}
        else if (Str == "piping") {compt_piping++;}
        else if (Str == "notbees") {compt_notbees++;}
        else if (Str == "voices") {compt_voices++;}
    }

    if ( (compt_piping > compt_normal) && (compt_piping > compt_notbees) && (compt_piping > compt_voices) ) {resultPredictions = 22;}
    
    else if ( (compt_normal >= compt_piping) && (compt_normal >= compt_notbees) && (compt_normal >= compt_voices) ) {resultPredictions = 33;}
    
    else if ( (compt_notbees > compt_piping) && (compt_notbees > compt_piping) && (compt_notbees > compt_voices) ) {resultPredictions = 44;}
    
    else if ( (compt_voices > compt_piping) && (compt_voices > compt_normal) && (compt_voices > compt_notbees) ) {resultPredictions = 55;}

    else {resultPredictions = 13;} 
        
    if (EI_CLASSIFIER_HAS_ANOMALY == 1) {resultPredictions = 14;}
 
    Serial.print("compteurs : normal ");
    Serial.print(compt_normal, DEC);
    Serial.print(", piping ");
    Serial.print(compt_piping, DEC);
    Serial.print("\n");
    return resultPredictions;
}

Data fctCapteursAcquisition (int batterie) {
  Data donnees;
  //phase de mise dans la structure
  donnees.HumiditeExterne = fctHumidityExt();
  donnees.TempExterne = fctTempExt();
  threetemperatures result = fctTempInt();
  donnees.TempInt = result.tempInt1;
  donnees.TempInt2 = result.tempInt2;
  donnees.TempInt3 = result.tempInt3;
  donnees.Poids = fctPoids();
  
  donnees.Batterie = batterie;
  
  donnees.Luminosite = fctLuminosite();
  donnees.Audio = fctAudio();
  donnees.HumiditeInterne = fctHumidityInt();
  return donnees;
}

void affichageCapteurs(Data donnees) {
  // Print sur le terminal pour visualiser
  Serial.print(F("Humidite externe : "));
  Serial.print(donnees.HumiditeExterne,HEX);
  Serial.print("%\r\n");
  Serial.print("Temperature Externe : ");
  Serial.print(donnees.TempExterne,HEX);
  Serial.print("°\r\n");
  Serial.print("Temperature Interne : ");
  Serial.print(donnees.TempInt,HEX);
  Serial.print("°C");
  Serial.print("\r\n");
  Serial.print("Temperature Interne 2 : ");
  Serial.print(donnees.TempInt2,HEX);
  Serial.print("°C");
  Serial.print("\r\n");
  Serial.print("Temperature Interne 3 : ");
  Serial.print(donnees.TempInt3,HEX);
  Serial.print("°C");
  Serial.print("\r\n");
  Serial.print("Poids : ");
  Serial.print(donnees.Poids,HEX);
  Serial.println("kg");
  Serial.print("Humidité Interne : ");
  Serial.println(donnees.HumiditeInterne,HEX);
  Serial.print("Batterie : ");
  Serial.println(donnees.Batterie,HEX);
  Serial.print("Luminosite : ");
  Serial.println(donnees.Luminosite,HEX);
  Serial.print("Audio : ");
  Serial.println(donnees.Audio,HEX);
}

//--------------------------------------------Loop--------------------------------------------

void loop() {
  
  BonEnvoi Bonnesdonnees;

//  TEST LUNDI 3 JANVIER
  int batterie = fctNiveau_batterie();
  if (batterie >= 10) {
    Data donnees = fctCapteursAcquisition(batterie);
  
    // On remplit les 7 premiers bits avec les donnees de l'humidité externe
    Bonnesdonnees.Envoi = donnees.HumiditeExterne;
    Bonnesdonnees.Envoi = Bonnesdonnees.Envoi | (donnees.TempInt2 << 8); 
    Bonnesdonnees.Envoi = Bonnesdonnees.Envoi | (donnees.TempExterne << 16);
    Bonnesdonnees.Envoi = Bonnesdonnees.Envoi | (donnees.TempInt << 24);
    
    Bonnesdonnees.Envoi2 = donnees.Poids;
    Bonnesdonnees.Envoi2 = Bonnesdonnees.Envoi2 | (donnees.HumiditeInterne << 16);
    Bonnesdonnees.Envoi2 = Bonnesdonnees.Envoi2 | (donnees.TempInt3 << 24); 
  
    Bonnesdonnees.Envoi3 = donnees.Batterie;
    Bonnesdonnees.Envoi3 = Bonnesdonnees.Envoi3 | (donnees.Luminosite << 8);
    Bonnesdonnees.Envoi3 = Bonnesdonnees.Envoi3 | (donnees.Audio << 16);
  
    affichageCapteurs(donnees);
    Serial.println(Bonnesdonnees.Envoi,HEX);
    Serial.println(Bonnesdonnees.Envoi2,HEX);
    Serial.println(Bonnesdonnees.Envoi3,HEX);
  
    //Envoi données
    Serial1.write("AT$SF=");
    Serial1.print(Bonnesdonnees.Envoi,HEX); 
    Serial1.print(Bonnesdonnees.Envoi2,HEX);
    Serial1.print(Bonnesdonnees.Envoi3,HEX);
    Serial1.write("\r\n");

  }
  delay(1*60*10000UL);
  //delay(1*60*5000UL);
  //delay(30000);

}
