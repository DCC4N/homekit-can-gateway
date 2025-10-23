/**********************
Programe PortailGateway : HomeKit (iPhone) - Can pour mon portail jardin
HomeSpan: A HomeKit implementation for the ESP32   
------------------------------------------------   
from Example 13: Target States and Current States     
implementing a Garden Door Opener  
*********************/

#include "HomeSpan.h"
#include <ACAN_ESP32.h>

// ===============================
#define PHAUT    " ________________________________________________ "
#define PROJECT  "      PASSERELLE HOMEKIT-CAN POUR PORTAIL "
#define PVERSION "       VERSION 0.0.4 du 23 octobre 2025   "
#define PBAS     " ________________________________________________ "
#define PAUTHOR "dominique@locoduino.org"

// ========== MODE DEBUG ==========
#define DEBUG_MODE false  // Mettre à false pour désactiver les logs détaillés

// ========== CONFIGURATION CAN ==========
static const uint32_t CAN_BITRATE = 125000; // 125 kbps - à adapter
static const gpio_num_t CAN_TX_PIN = GPIO_NUM_23; // CAN TX pour MCP2562 1=TX
static const gpio_num_t CAN_RX_PIN = GPIO_NUM_22; // CAN RX pour MCP2562 4=RX
static const gpio_num_t LED_CLOSE_PIN = GPIO_NUM_16;
static const gpio_num_t LED_OPEN_PIN = GPIO_NUM_17;

// ========== TIMEOUT ==========
#define TIMEOUT_CAN_MS 15000  // 15 secondes (> 10s de répétition)

// ========== MACRO DEBUG ==========
#define DEBUG_PRINT(x) if(DEBUG_MODE) Serial.print(x)
#define DEBUG_PRINTLN(x) if(DEBUG_MODE) Serial.println(x)
#define DEBUG_PRINTF(x, y) if(DEBUG_MODE) Serial.print(x, y)
#define DEBUG_PRINTLNF(x, y) if(DEBUG_MODE) Serial.println(x, y)

// ========== IDENTIFIANTS CAN (À ADAPTER) ==========
// Messages envoyés au portail (commandes)
#define CAN_ID_CMD_OUVRIR      0x200
#define CAN_ID_CMD_FERMER      0x201

// Messages reçus du portail (états)
#define CAN_ID_ETAT_PORTAIL    0x100

// Valeurs d'état reçues du portail via CAN dans data[0]
#define ETAT_OUVERT     0
#define ETAT_FERME      1
#define ETAT_OUVRANT    2
#define ETAT_FERMANT    3
#define ETAT_OBSTACLE   4

// ========== FREERTOS ==========
TaskHandle_t tacheCANHandle = NULL;
SemaphoreHandle_t mutexHomeKit = NULL;

// ========== SERVICE GARAGEDOOROPENER HOMEKIT ==========
struct PortailCAN : Service::GarageDoorOpener {
  
  SpanCharacteristic *currentState;        // État actuel
  SpanCharacteristic *targetState;         // État cible (commande)
  SpanCharacteristic *obstructionDetected; // Détection d'obstacle
  
  unsigned long dernierMessageCAN;       // Timestamp du dernier message CAN reçu
  bool timeoutActif;                     // Flag de timeout
  
  PortailCAN() : Service::GarageDoorOpener() {
    
    currentState = new Characteristic::CurrentDoorState(1);  // 1 = CLOSED au démarrage
    targetState = new Characteristic::TargetDoorState(1);    // 1 = CLOSED au démarrage
    obstructionDetected = new Characteristic::ObstructionDetected(false);
    
    dernierMessageCAN = millis();
    timeoutActif = false;
    
    Serial.println("Service GarageDoorOpener configuration OK ");
    DEBUG_PRINTLN("  - Mode debug actif");
    DEBUG_PRINTLN("  - Tache FreeRTOS active");
  }
  
  // Callback appelé quand HomeKit envoie une commande
  boolean update() {
    
    int nouvelleCommande = targetState->getNewVal();
    int ancienneCommande = targetState->getVal();
    
    Serial.print(" COMMANDE HOMEKIT ");
    //Serial.print("  Ancienne cible: ");
    //Serial.println(ancienneCommande == 0 ? "OUVRIR" : "FERMER");
    //Serial.print("  Nouvelle cible: ");
    Serial.println(nouvelleCommande == 0 ? "OUVRIR" : "FERMER");
    
    // Envoi de la commande sur le bus CAN
    if (nouvelleCommande == 0) {
      // Commande OUVRIR
      //Serial.println(" Envoi commande CAN: OUVRIR");
      envoyerCommandeCAN(CAN_ID_CMD_OUVRIR, 1);
      digitalWrite(LED_CLOSE_PIN, LOW);
      digitalWrite(LED_OPEN_PIN, HIGH);
    } else {
      // Commande FERMER
      //Serial.println(" Envoi commande CAN: FERMER");
      envoyerCommandeCAN(CAN_ID_CMD_FERMER, 1);
      digitalWrite(LED_CLOSE_PIN, HIGH);
      digitalWrite(LED_OPEN_PIN, LOW);
    }
    
    return true; // Commande acceptée
  }
  
  // Mise à jour de l'état HomeKit depuis les messages CAN (thread-safe)
  void mettreAJourDepuisCAN(uint8_t etatCAN) {
    
    // Réinitialiser le timeout
    dernierMessageCAN = millis();
    if (timeoutActif) {
      Serial.println("✓ Communication CAN rétablie");
      timeoutActif = false;
    }
    
    int nouvelEtatCurrent = -1;
    int nouvelEtatTarget = -1;  // AJOUT: pour synchroniser le switch HomeKit
    bool obstacleDetecte = false;
    const char* nomEtat = "";
    
    // Traduction état CAN → état HomeKit
    switch(etatCAN) {
      case ETAT_FERME:
        nouvelEtatCurrent = 1; // CLOSED
        nouvelEtatTarget = 1;  // Le portail est fermé → switch sur "Fermé"
        nomEtat = "FERMÉ";
        break;
        
      case ETAT_OUVRANT:
        nouvelEtatCurrent = 2; // OPENING
        nouvelEtatTarget = 0;  // En cours d'ouverture → cible = OUVERT
        nomEtat = "OUVERTURE";
        break;
        
      case ETAT_OUVERT:
        nouvelEtatCurrent = 0; // OPEN
        nouvelEtatTarget = 0;  // Le portail est ouvert → switch sur "Ouvert"
        nomEtat = "OUVERT";
        break;
        
      case ETAT_FERMANT:
        nouvelEtatCurrent = 3; // CLOSING
        nouvelEtatTarget = 1;  // En cours de fermeture → cible = FERMÉ
        nomEtat = "FERMETURE";
        break;
        
      case ETAT_OBSTACLE:
        obstacleDetecte = true;
        nouvelEtatCurrent = 4; // STOPPED (obstacle)
        // Ne pas changer targetState en cas d'obstacle pour conserver l'intention
        nomEtat = "OBSTACLE";
        break;
        
      default:
        Serial.print("Etat CAN inconnu: 0x");
        Serial.println(etatCAN, HEX);
        return;
    }
    
    Serial.print("┌── ETAT CAN RECU ───┐\n");
    Serial.print("  Etat portail: ");
    Serial.println(nomEtat);
    DEBUG_PRINT("  Valeur CAN brute: 0x");
    DEBUG_PRINTLNF(etatCAN, HEX);
    
    // SECTION CRITIQUE : Mise à jour HomeKit (thread-safe)
    if (xSemaphoreTake(mutexHomeKit, portMAX_DELAY) == pdTRUE) {
      
      // Mise à jour de currentState
      if (nouvelEtatCurrent != -1) {
        int ancienEtat = currentState->getVal();
        if (ancienEtat != nouvelEtatCurrent) {
          currentState->setVal(nouvelEtatCurrent);
          Serial.print("  → HomeKit CurrentState: ");
          Serial.print(ancienEtat);
          Serial.print(" → ");
          Serial.println(nouvelEtatCurrent);
        } else {
          DEBUG_PRINTLN("  (Etat HomeKit inchangé)");
        }
      }
      
      // CORRECTION: Mise à jour de targetState pour synchroniser le switch
      if (nouvelEtatTarget != -1) {
        int ancienneTarget = targetState->getVal();
        if (ancienneTarget != nouvelEtatTarget) {
          targetState->setVal(nouvelEtatTarget);
          Serial.print("  → HomeKit TargetState: ");
          Serial.print(ancienneTarget);
          Serial.print(" → ");
          Serial.println(nouvelEtatTarget);
        }
      }
      
      bool ancienObstacle = obstructionDetected->getVal();
      if (ancienObstacle != obstacleDetecte) {
        obstructionDetected->setVal(obstacleDetecte);
        Serial.print("  → Obstacle détecté: ");
        Serial.println(obstacleDetecte ? "OUI" : "NON");
      }
      
      xSemaphoreGive(mutexHomeKit);
    } else {
      Serial.println("  ⚠ ERREUR: Impossible de verrouiller mutex HomeKit");
    }
  }
  
  // Vérification du timeout de communication CAN
  void verifierTimeout() {
    unsigned long maintenant = millis();
    
    // Gestion du débordement de millis() (toutes les ~49 jours)
    if (maintenant < dernierMessageCAN) {
      dernierMessageCAN = maintenant;
      return;
    }
    
    if ((maintenant - dernierMessageCAN) > TIMEOUT_CAN_MS) {
      if (!timeoutActif) {
        Serial.println("----------------------------------------------------");
        Serial.println("TIMEOUT: Aucun message CAN depuis 15s");
        Serial.println("  Verifiez la connexion au portail  ");
        Serial.println("----------------------------------------------------");
        timeoutActif = true;
      }
    }
  }
  
private:
  // Fonction d'envoi de commande CAN
  void envoyerCommandeCAN(uint32_t canId, uint8_t valeur) {
    CANMessage message;
    message.id = canId;
    message.len = 1;
    message.data[0] = valeur;
    message.ext = false; // Standard CAN (11 bits)
    message.rtr = false;
    
    DEBUG_PRINT("  CAN TX - ID: 0x");
    DEBUG_PRINTF(canId, HEX);
    DEBUG_PRINT(" Data[0]: 0x");
    DEBUG_PRINTF(valeur, HEX);
    DEBUG_PRINTLN("");
    
    if (ACAN_ESP32::can.tryToSend(message)) {
      Serial.println(" Message CAN envoye avec succes");
    } else {
      Serial.println(" ERREUR: Echec envoi message CAN");
      //Serial.println(" Buffer CAN plein ou bus sature");
    }
  }
};

// ========== INSTANCE GLOBALE DU SERVICE ==========
PortailCAN *monPortail;

// ========== TÂCHE FREERTOS : ÉCOUTE BUS CAN ==========
void tacheEcouteCAN(void *parametres) {
  
  Serial.println("[Tâche CAN] Démarrage sur Core " + String(xPortGetCoreID()));
  
  CANMessage messageRecu;
  
  while(true) {
    
    // Lecture des messages CAN (bloquante avec timeout)
    if (ACAN_ESP32::can.receive(messageRecu)) {
      
      DEBUG_PRINTLN("------------------------------------");
      DEBUG_PRINT("[Tache CAN] Message recu:\n");
      DEBUG_PRINT("  ID: 0x");
      DEBUG_PRINTF(messageRecu.id, HEX);
      DEBUG_PRINT("  Len: ");
      DEBUG_PRINTLN(messageRecu.len);
      DEBUG_PRINT("  Data: ");
      for(int i = 0; i < messageRecu.len; i++) {
        DEBUG_PRINT("0x");
        DEBUG_PRINTF(messageRecu.data[i], HEX);
        DEBUG_PRINT(" ");
      }
      DEBUG_PRINTLN("");
      
      // Filtrage des messages d'état du portail
      if (messageRecu.id == CAN_ID_ETAT_PORTAIL && messageRecu.len > 0) {
        uint8_t etatPortail = messageRecu.data[0]; // etat FERME, OUVRANT, OUVERT, FERMANT, OBSTACLE
        
        // Mise à jour de HomeKit (thread-safe)
        monPortail->mettreAJourDepuisCAN(etatPortail);
      } else {
        DEBUG_PRINTLN("  → Message ignoré (ID non surveillé)");
      }
    }
    
    // Vérification du timeout
    monPortail->verifierTimeout();
    
    // Délai pour éviter de surcharger le CPU (10ms)
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ========== SETUP ==========
void setup() {
  
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n");
  Serial.println(PHAUT);
  Serial.println(PROJECT);
  Serial.println(PVERSION);
  Serial.println(PBAS);
  Serial.println();
  pinMode(LED_CLOSE_PIN, OUTPUT);
  pinMode(LED_OPEN_PIN, OUTPUT);
  digitalWrite(LED_CLOSE_PIN, HIGH);
  
  // Création du mutex pour accès thread-safe à HomeKit
  mutexHomeKit = xSemaphoreCreateMutex();
  if (mutexHomeKit == NULL) {
    Serial.println("ERREUR CRITIQUE: Impossible de creer le mutex");
    while(1);
  }
  Serial.println("Mutex HomeKit creation OK");
  
  // Configuration du bus CAN
  Serial.println("\n[ Configuration CAN ]");
  Serial.print("  Bitrate: ");
  Serial.print(CAN_BITRATE);
  Serial.println(" bps");
  Serial.print("  TX Pin: GPIO");
  Serial.println(CAN_TX_PIN);
  Serial.print("  RX Pin: GPIO");
  Serial.println(CAN_RX_PIN);
  
  ACAN_ESP32_Settings canSettings(CAN_BITRATE);
  canSettings.mRxPin = CAN_RX_PIN;
  canSettings.mTxPin = CAN_TX_PIN;
  
  uint32_t errorCode = ACAN_ESP32::can.begin(canSettings);
  if (errorCode == 0) {
    Serial.println(" Bus CAN initialise avec succes");
  } else {
    Serial.print(" ERREUR configuration CAN: 0x");
    Serial.println(errorCode, HEX);
    Serial.println(" Verifiez les connexions CAN et redemarrez");
    while(1); // Arrêt si erreur CAN
  }
  
  // Configuration HomeSpan
  Serial.println("\n[ Configuration HomeSpan ]");
  homeSpan.enableOTA();
  homeSpan.begin(Category::GarageDoorOpeners, "Portail Jardin");
  
  // Création de l'accessoire HomeKit
  new SpanAccessory();
    new Service::AccessoryInformation();
      new Characteristic::Identify();
      new Characteristic::Manufacturer("DIY");
      new Characteristic::Model("ESP32-CAN-FreeRTOS");
      new Characteristic::Name("Portail Jardin");
      new Characteristic::SerialNumber("CAN-002");
      new Characteristic::FirmwareRevision("2.1");
    
    monPortail = new PortailCAN(); // Service GarageDoorOpener
  
  // Création de la tâche FreeRTOS pour écouter le bus CAN
  Serial.println("\n[ Configuration FreeRTOS ]");
  BaseType_t resultat = xTaskCreatePinnedToCore(
    tacheEcouteCAN,        // Fonction de la tâche
    "TacheCAN",            // Nom de la tâche
    4096,                  // Taille de la stack (4KB)
    NULL,                  // Paramètres
    2,                     // Priorité (2 = moyenne, HomeSpan tourne à 1)
    &tacheCANHandle,       // Handle de la tâche
    1                      // Core 1 (Core 0 réservé pour WiFi/HomeSpan)
  );
  
  if (resultat == pdPASS) {
    Serial.println(" Tâche CAN creee sur Core 1");
    Serial.print(" Priorité: 2 (HomeSpan: 1)\n");
  } else {
    Serial.println(" ERREUR: Impossible de creer la tache CAN");
    while(1);
  }
  
  Serial.println();
  Serial.println(" _________________________________________ ");
  Serial.println("   SYSTEME PRET                      ");
  Serial.println("   HomeSpan: Core 0 | CAN: Core 1    ");
  Serial.println("   En attente de connexion HomeKit...");
  Serial.println(" _________________________________________ ");
  Serial.println();
}

// ========== LOOP ==========
void loop() {
  
  // SECTION CRITIQUE : Gestion HomeSpan (thread-safe)
  if (xSemaphoreTake(mutexHomeKit, portMAX_DELAY) == pdTRUE) {
    homeSpan.poll();
    xSemaphoreGive(mutexHomeKit);
  }
  
  // Petit délai (HomeSpan gère déjà son timing interne)
  delay(10);
}
