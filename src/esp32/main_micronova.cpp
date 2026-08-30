// ============================================================================
//  main_micronova.cpp
//  ----------------------------------------------------------------------------
//  ESP32-C3 / carte de commande Micronova (poele a granulés)
//
//  MILESTONE 2 : mode interrogateur (l'ESP devient maître du bus)
//    - la carte Micronova est un ESCLAVE : elle ne repond qu'a une sollicitation
//    - l'ESP envoie des requetes de lecture  [0x00|0x20] [adresse]
//    - le poele repond 2 octets [checksum] [valeur], checksum = adresse+valeur
//    - delai de reponse ~120 ms (mesure par eni23/micronova-controller)
//    - affiche les valeurs decodees toutes les 5 s sur le Serial (moniteur USB)
//
//  Remarque historique (MILESTONE 1, sniffer passif) : sans console filaire ni
//  module wifi actif sur le bus, il n'y a AUCUN trafic spontane -> d'ou le
//  besoin de ce mode interrogateur.
//
//  Protocole Micronova (doc ESPHome "micronova" + eni23 + ridiculouslab.com) :
//    1200 bauds, 8 bits de données, pas de parite, 2 stop bits (8N2)
//    le bus entre la carte et la console est une ligne série unifilaire.
//
//  Câblage optocoupleurs (2 canaux, montage NON inverse) :
//    RX  : poele data -> resistance ~1k -> diode RX opto -> GND poele
//          phototransistor en emetteur-suiveur :
//          collecteur -> +3.3V ESP, emetteur -> GPIO20, PULLDOWN sur GPIO20
//          (idle haut du bus -> GPIO20 haut, pas d'inversion RX)
//    TX  : GPIO21 -> resistance ~470 -> cathode LED TX opto, anode -> +3.3V
//          LED active BAS (allume quand GPIO21 passe bas = start bit),
//          donc eteinte au repos, pas d'inversion TX
//          phototransistor TX : collecteur -> data poele, emetteur -> GND poele
//    Semi-duplex sans opto enable_rx : l'echo de nos requetes est draine en
//    logiciel (ECHO_DRAIN_MS) apres chaque emission.
//
//  Moniteur : Serial USB (CDC) 115200 bauds (ARDUINO_USB_CDC_ON_BOOT=1).
// ============================================================================

#include <Arduino.h>
#include "esp_task_wdt.h"        // pour nourrir le watchdog pendant le scan

// ---------------------------------------------------------------------------
//  Configuration
// ---------------------------------------------------------------------------
#define PIN_STOVE_RX       20               // UART0 RX de l'ESP32-C3
#define PIN_STOVE_TX       21               // UART0 TX
#define STOVE_BAUD         1200             // vitesse du bus Micronova
#define STOVE_CONFIG       SERIAL_8N2       // 8 data / no parity / 2 stop
#define DEBUG_INTERVAL_MS  5000UL           // bloc de debug toutes les 5 s
#if !STOVE_USE_TX
#define FRAME_GAP_MS       40UL             // silence entre 2 trames (a 1200
                                            // bauds, 1 octet ~= 9 ms)
#define FRAME_MAX_BYTES    64               // taille max d'une trame stockée
#endif

// --- Options du mode diagnostic ---
#define STOVE_USE_TX        1                // 1 = mode interrogateur (TX actif)
// Inversion UART separée TX / RX. Pour un câblage opto NON inverse
// (RX emetteur-suiveur 3.3V + LED TX active-bas) les deux sont a 0.
#define STOVE_INVERT_TX     0                // 1 = inverser la voie TX (GPIO21)
#define STOVE_INVERT_RX     0                // 1 = inverser la voie RX (GPIO20)
// Mode de tirage du pin RX : INPUT_PULLDOWN pour emetteur-suiveur,
// INPUT_PULLUP pour un montage collecteur ouvert.
#define STOVE_RX_PULLMODE   INPUT_PULLDOWN
#define DIAGNOSTIC_MODE     1                // 1 = bloc de diagnostic 5 s
#define DIAG_IDLE_MS        100UL            // duree du test de niveau idle
#define WAVEFORM_SAMPLES    256              // nb echantillons de la waveform
#define WAVEFORM_STEP_US    30UL             // pas d'echantillonnage (us)

// --- Test d'echo (validation du circuit opto en boucle) ---
// 1 = a chaque cycle, envoyer une rafale connue sur TX et compter les octets
// reçus sur RX (echo). Permet de valider la chaine RX/TX sans le poele,
// en reliant le collecteur TX a l'anode LED RX via un fil (loopback).
#define ECHO_TEST           0

// --- Scanner (recherche automatique de la bonne config UART) ---
// 1 = lancer un scan des combinaisons baud/config/inversion au boot,
// puis balayage des adresses RAM/EEPROM avec la meilleure config trouvee.
// A tout moment, un menu serie permet de relancer / changer la config.
#define STOVE_SCAN_MODE     1               // 1 = scan configs au demarrage
#define SCAN_LISTEN_MS      400UL           // ecoute passive par combinaison
#define SCAN_PROBE_MS       220UL           // fenetre de lecture par sonde
#define SCAN_ADDR_MAX       0x7F            // derniere adresse de balayage

// --- Options de l'interrogation ---
#define ECHO_DRAIN_MS       40UL             // duree du drain de notre echo
                                             // (2 octets ~ 19 ms a 1200 bauds)
#define REPLY_TIMEOUT_MS    300UL            // delai max pour collecter reponse
                                             // (le poele repond en ~120 ms)

// Adresses a interroger (mapping eni23 / ESPHome, a ajuster pour MUSA C/AIR).
// location : 0x00 = RAM (lecture), 0x20 = EEPROM (lecture)
typedef struct {
  uint8_t  location;
  uint8_t  address;
  const char* label;
} StoveRead;

static const StoveRead stoveReads[] = {
  { 0x00, 0x21, "etat"           },
  { 0x00, 0x01, "temp. ambiante" },
  { 0x00, 0x3E, "temp. fumees"   },
  { 0x00, 0x34, "puissance"      },
  { 0x00, 0x37, "rpm ventilo"    },
  { 0x20, 0x7D, "thermostat EEP" },
}; 
#define STOVE_READS_COUNT (sizeof(stoveReads) / sizeof(stoveReads[0]))

// Combinaisons d'UART a scanner (vitesse x format x inversion RX).
typedef struct {
  uint32_t     baud;
  uint32_t     config;
  const char*  label;
} ScanConfig;

static const ScanConfig scanConfigs[] = {
  { 1200, SERIAL_8N2, "1200/8N2" },
  { 1200, SERIAL_8N1, "1200/8N1" },
  { 2400, SERIAL_8N2, "2400/8N2" },
  { 2400, SERIAL_8N1, "2400/8N1" },
  { 4800, SERIAL_8N2, "4800/8N2" },
  { 4800, SERIAL_8N1, "4800/8N1" },
  { 9600, SERIAL_8N2, "9600/8N2" },
  { 9600, SERIAL_8N1, "9600/8N1" },
};
#define SCAN_CONFIGS_COUNT (sizeof(scanConfigs) / sizeof(scanConfigs[0]))

// Adresses sondees pour evaluer chaque combinaison (RAM loc 0x00 + EEPROM 0x20)
static const uint8_t scanProbeAddrs[] = { 0x21, 0x01, 0x3E, 0x7D };
#define SCAN_PROBE_ADDRS_COUNT (sizeof(scanProbeAddrs) / sizeof(scanProbeAddrs[0]))

// ---------------------------------------------------------------------------
//  Variables globales
// ---------------------------------------------------------------------------
// --- Variables du mode passif (sniffer) ---
#if !STOVE_USE_TX
uint8_t  frameData[FRAME_MAX_BYTES];
uint16_t frameLen = 0;
bool     frameOverflow = false;
uint32_t lastByteMs = 0;
uint32_t totalFrames  = 0;
uint32_t windowFrames = 0;
#endif

uint32_t totalBytes   = 0;
uint32_t windowBytes  = 0;   // octets reçus depuis le dernier bloc de debug
uint32_t lastDebugMs  = 0;

// --- Variables du mode diagnostic ---
bool     idleLevelHigh   = true;            // niveau de repos de la ligne RX
uint32_t diagTransitions = 0;               // transitions detectees (fenetre)
uint32_t minHighUs = 0xFFFFFFFF, maxHighUs = 0;  // largeurs impulsions HAUT
uint32_t minLowUs  = 0xFFFFFFFF, maxLowUs  = 0;  // largeurs impulsions BAS
bool     lastLevel         = false;         // dernier niveau lu
uint32_t lastChangeUs      = 0;             // instant du dernier changement
const char* rxPullStr = "pull-up";          // mode reel du pin RX (affiche)

// --- Variables du mode interrogateur ---
uint8_t  stoveValues[STOVE_READS_COUNT];
bool     stoveValid[STOVE_READS_COUNT];
uint8_t  stoveRaw[STOVE_READS_COUNT][2];    // octets bruts reponses (2 max)
uint8_t  stoveRawLen[STOVE_READS_COUNT];
uint32_t stovePollOk  = 0;                  // lecture conforme (checksum ok)
uint32_t stovePollBad = 0;                  // lecture non conforme ou absence
bool     stoveAnyReply = false;             // au moins une reponse 2 octets

// --- Configuration UART runtime (modifiable par le menu / le scanner) ---
uint32_t gBaud     = STOVE_BAUD;            // vitesse courante
uint32_t gConfig   = STOVE_CONFIG;          // format courant (8N1/8N2...)
bool     gTxInvert = (STOVE_INVERT_TX != 0); // inversion TX courante
bool     gRxInvert = (STOVE_INVERT_RX != 0); // inversion RX courante
bool     scanBusy  = false;                 // un scan est en cours

// ---------------------------------------------------------------------------
//  Affichage d'un octet en hex (2 chiffres)
// ---------------------------------------------------------------------------
void printHex(uint8_t b) {
  static const char hexDigits[] = "0123456789ABCDEF";
  Serial.print(hexDigits[b >> 4]);
  Serial.print(hexDigits[b & 0x0F]);
}

#if !STOVE_USE_TX
// ---------------------------------------------------------------------------
//  Affichage d'une trame complete (hex + ascii)
// ---------------------------------------------------------------------------
void printFrame() {
  if (frameLen == 0) {
    return;
  }

  totalFrames++;
  windowFrames++;

  Serial.print("[T");
  if (totalFrames < 1000) Serial.print('0');
  if (totalFrames < 100)  Serial.print('0');
  if (totalFrames < 10)   Serial.print('0');
  Serial.print(totalFrames);
  Serial.print("] ");
  Serial.print(frameLen);
  Serial.print(" octets  hex: ");

  for (uint16_t i = 0; i < frameLen; i++) {
    printHex(frameData[i]);
    Serial.print(' ');
  }

  if (frameOverflow) {
    Serial.print("(! depassement >64 octets) ");
  }

  Serial.print(" ascii: ");
  for (uint16_t i = 0; i < frameLen; i++) {
    char c = (frameData[i] >= 0x20 && frameData[i] <= 0x7E)
                 ? (char)frameData[i]
                 : '.';
    Serial.print(c);
  }
  Serial.println();

  frameLen = 0;
  frameOverflow = false;
}
#endif // !STOVE_USE_TX

// ---------------------------------------------------------------------------
//  Diagnostic : niveau de repos de la ligne RX
//  (lecture physique du pad, independante du decodage UART)
// ---------------------------------------------------------------------------
void detectIdleLevel() {
  uint32_t start = millis();
  uint32_t highCount = 0, lowCount = 0;
  while (millis() - start < DIAG_IDLE_MS) {
    if (digitalRead(PIN_STOVE_RX)) {
      highCount++;
    } else {
      lowCount++;
    }
  }
  idleLevelHigh = (highCount > lowCount);
}

// ---------------------------------------------------------------------------
//  Diagnostic : capture de la forme du signal (ASCII)
//  '_' = niveau bas, '#' = niveau haut, ~7.7 ms de signal au total
// ---------------------------------------------------------------------------
void printWaveform() {
  Serial.println("  waveform (~30us/pas, _=BAS #=HAUT):");
  for (uint16_t i = 0; i < WAVEFORM_SAMPLES; i++) {
    if (i % 64 == 0) {
      Serial.print("    ");
    }
    Serial.print(digitalRead(PIN_STOVE_RX) ? '#' : '_');
    if ((i + 1) % 64 == 0) {
      Serial.println();
    }
    delayMicroseconds(WAVEFORM_STEP_US);
  }
  Serial.println();
}

// ---------------------------------------------------------------------------
//  Diagnostic : comptage et mesure des transitions du signal (appele loop)
// ---------------------------------------------------------------------------
void updateDiag(uint32_t nowUs) {
  bool level = digitalRead(PIN_STOVE_RX);
  if (level != lastLevel) {
    uint32_t dur = nowUs - lastChangeUs;
    if (dur > 0) {
      if (lastLevel) {                    // etat precedent = HAUT
        if (dur < minHighUs) minHighUs = dur;
        if (dur > maxHighUs) maxHighUs = dur;
      } else {                            // etat precedent = BAS
        if (dur < minLowUs) minLowUs = dur;
        if (dur > maxLowUs) maxLowUs = dur;
      }
    }
    lastLevel = level;
    lastChangeUs = nowUs;
    diagTransitions++;
  }
}

// ---------------------------------------------------------------------------
//  Lecture d'un octet sur Serial0 avec compteurs (idx tableau valeurs)
// ---------------------------------------------------------------------------
int robustReadByte() {
  if (!Serial0.available()) {
    return -1;
  }
  int b = Serial0.read();
  if (b >= 0) {
    totalBytes++;
    windowBytes++;
  }
  return b;
}

// ---------------------------------------------------------------------------
//  Nom lisible de l'etat du poele
// ---------------------------------------------------------------------------
const char* stoveStateName(uint8_t s) {
  switch (s) {
    case 0:  return "Off";
    case 1:  return "Start";
    case 2:  return "Pellet load";
    case 3:  return "Ignition";
    case 4:  return "Work";
    case 5:  return "Brazier clean";
    case 6:  return "Final clean";
    case 7:  return "Standby";
    case 8:  return "Pellet missing";
    case 9:  return "Ignition fail";
    default: return "?";
  }
}

// ---------------------------------------------------------------------------
//  Lecture d'une variable du poele (requete 2 octets, reponse 2 octets)
//  Retourne true si la reponse a pu etre collectee (conforme ou non).
// ---------------------------------------------------------------------------
bool stoveReadOne(uint8_t location, uint8_t addr, uint8_t& value,
                  uint8_t rawOut[2], uint8_t& rawLen, bool& checksumOk) {
  rawLen = 0;
  checksumOk = false;

  // 1) Purge de la file RX (echos precedents eventuels)
  while (robustReadByte() >= 0) { /* drainer */ }

  // 2) Emission de la requete [location] [address]
  Serial0.write(location);
  Serial0.write(addr);
  Serial0.flush();

  // 3) Drain de notre echo (semi-duplex sans opto enable_rx)
  {
    uint32_t t0 = millis();
    while (millis() - t0 < ECHO_DRAIN_MS) {
      while (robustReadByte() >= 0) { /* drainer */ }
    }
  }

  // 4) Attente de la reponse du poele (~120 ms) puis collecte
  {
    uint32_t t0 = millis();
    while (millis() - t0 < REPLY_TIMEOUT_MS) {
      int b = robustReadByte();
      if (b >= 0 && rawLen < 2) {
        rawOut[rawLen++] = (uint8_t)b;
      }
      if (rawLen >= 2) {
        break;
      }
    }
  }

  if (rawLen < 2) {
    return false;                       // pas de reponse exploitable
  }

  value = rawOut[1];                    // 2eme octet = valeur
  checksumOk = ((uint8_t)(addr + value) == rawOut[0]);
  return true;
}

// ---------------------------------------------------------------------------
//  Cycle de lecture de toutes les variables configurees
// ---------------------------------------------------------------------------
void stovePollAll() {
  stoveAnyReply = false;

  for (uint8_t i = 0; i < STOVE_READS_COUNT; i++) {
    bool ck = false;
    bool got = stoveReadOne(stoveReads[i].location, stoveReads[i].address,
                            stoveValues[i], stoveRaw[i], stoveRawLen[i], ck);
    stoveValid[i] = got;
    if (got) {
      stoveAnyReply = true;
      stovePollOk++;
    } else {
      stovePollBad++;
    }
  }

  Serial.print("[poll] requetes envoyees, ");
  Serial.print(stovePollOk);
  Serial.print(" conforme(s), ");
  Serial.print(stovePollBad);
  Serial.println(" non-conforme(s)/absente(s)");
}

// ---------------------------------------------------------------------------
//  Test d'echo : envoie une rafale connue et compte les octets reçus (echo).
//  Utile pour valider la chaine opto RX/TX en boucle, sans le poele :
//  relier collecteur du TX opto -> anode LED du RX opto (+ GND commun).
// ---------------------------------------------------------------------------
void echoTest() {
  const uint8_t pattern[] = { 0x00, 0x21, 0x00, 0x01, 0x00, 0x3E, 0x00, 0x34 };
  const uint8_t n = sizeof(pattern);

  // Purge
  while (Serial0.available()) {
    Serial0.read();
  }

  Serial.print("[echo] envoi ");
  Serial.print(n);
  Serial.print(" octets, attente echo... ");

  Serial0.write(pattern, n);
  Serial0.flush();

  uint32_t t0 = millis();
  uint32_t got = 0;
  while (millis() - t0 < 500) {
    while (Serial0.available()) {
      Serial0.read();
      got++;
    }
  }

  Serial.print("recu ");
  Serial.print(got);
  Serial.print(" octets sur ");
  Serial.println(n);
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
//  Application de la configuration UART courante (runtime)
//  re-applique baud, format, inversions et pull du pin RX.
// ---------------------------------------------------------------------------
void applyUartParams() {
  Serial0.end();
  Serial0.begin(gBaud, gConfig, PIN_STOVE_RX, PIN_STOVE_TX, gTxInvert);
  Serial0.setRxInvert(gRxInvert);
  pinMode(PIN_STOVE_RX, STOVE_RX_PULLMODE);
  rxPullStr = (STOVE_RX_PULLMODE == INPUT_PULLUP) ? "pull-up" : "pull-down";
  windowBytes = 0;
  diagTransitions = 0;
}

// ---------------------------------------------------------------------------
//  Impression de la config UART courante
// ---------------------------------------------------------------------------
void printUartParams() {
  Serial.print(gBaud);
  Serial.print(" ");
  Serial.print((gConfig == SERIAL_8N1) ? "8N1" : (gConfig == SERIAL_8N2) ? "8N2" : "?");
  Serial.print(" rxInv=");
  Serial.print(gRxInvert ? "1" : "0");
  Serial.print(" txInv=");
  Serial.print(gTxInvert ? "1" : "0");
  Serial.print(" pinRX=");
  Serial.print(rxPullStr);
}

// ---------------------------------------------------------------------------
//  Ecoute passive : compte les octets decodes sans rien emettre
//  (detecte un autre maitre actif sur le bus, ex. module wifi).
// ---------------------------------------------------------------------------
int scanListenPassive(uint32_t ms) {
  uint32_t t0 = millis();
  uint32_t count = 0;
  while (millis() - t0 < ms) {
    while (Serial0.available()) {
      Serial0.read();
      count++;
    }
    esp_task_wdt_reset();
    delay(1);
  }
  return (int)count;
}

// ---------------------------------------------------------------------------
//  Sonde : envoie une requete [location][addr] puis collecte ce qui revient.
//  ret = nombre d'octets recus (inclut notre echo si config correcte).
//  si rawLen>=4, raw[2],raw[3] = reponse potentielle [checksum][valeur].
// ---------------------------------------------------------------------------
int scanProbeOne(uint8_t location, uint8_t addr,
                 uint8_t raw[8], uint8_t& rawLen, bool& checksumOk) {
  // Purge
  while (Serial0.available()) {
    Serial0.read();
  }

  uint8_t q[2] = { location, addr };
  Serial0.write(q, 2);
  Serial0.flush();

  uint32_t t0 = millis();
  rawLen = 0;
  checksumOk = false;
  while (millis() - t0 < SCAN_PROBE_MS) {
    while (Serial0.available() && rawLen < 8) {
      raw[rawLen++] = (uint8_t)Serial0.read();
    }
    esp_task_wdt_reset();
    delay(1);
  }

  // Si on a recu notre echo [loc][addr] + reponse [chek][val], alors
  // raw[2] est le checksum et raw[3] la valeur -> checksum = addr + valeur.
  if (rawLen >= 4) {
    checksumOk = ((uint8_t)(addr + raw[3]) == raw[2]);
  }
  return (int)rawLen;
}
// ---------------------------------------------------------------------------
//  Scanner des combinaisons d'UART : baud x format x inversion RX
//  Chaque combinaison est evaluee par ecoute passive + sondes.
// ---------------------------------------------------------------------------
void stoveScanConfigs() {
  Serial.println();
  Serial.println("======= SCAN CONFIGS UART =======");
  Serial.println("combo        listen | p21 p01 p3E p7D | score");

  int      bestScore = -1;
  uint32_t bestBaud   = STOVE_BAUD;
  uint32_t bestConfig = STOVE_CONFIG;
  bool     bestRxInv  = (STOVE_INVERT_RX != 0);
  const char* bestLabel = "?";

  for (uint8_t ci = 0; ci < SCAN_CONFIGS_COUNT; ci++) {
    for (uint8_t inv = 0; inv < 2; inv++) {
      gBaud     = scanConfigs[ci].baud;
      gConfig   = scanConfigs[ci].config;
      gRxInvert = (inv == 1);
      applyUartParams();

      int listen = scanListenPassive(SCAN_LISTEN_MS);

      bool ck[SCAN_PROBE_ADDRS_COUNT];
      uint8_t raw[8]; uint8_t rl;
      int p[SCAN_PROBE_ADDRS_COUNT];
      int score = 0;

      for (uint8_t i = 0; i < SCAN_PROBE_ADDRS_COUNT; i++) {
        // P1/P2/P3 : RAM (loc 0x00), P4 : EEPROM (loc 0x20)
        uint8_t loc = (i == 3) ? 0x20 : 0x00;
        p[i] = scanProbeOne(loc, scanProbeAddrs[i], raw, rl, ck[i]);
        score += p[i];
        if (ck[i]) score += 20;
      }

      Serial.print(scanConfigs[ci].label);
      Serial.print(inv ? "  +RXinv " : "        ");
      Serial.print(" | ");
      if (listen < 100) Serial.print(' ');
      if (listen < 10)  Serial.print(' ');
      Serial.print(listen);
      Serial.print("    | ");
      for (uint8_t i = 0; i < SCAN_PROBE_ADDRS_COUNT; i++) {
        if (p[i] < 10) Serial.print(' ');
        Serial.print(p[i]);
        Serial.print(ck[i] ? "*" : " ");
        Serial.print(' ');
      }
      Serial.print(" | ");
      Serial.println(score);

      if (score > bestScore) {
        bestScore   = score;
        bestBaud    = scanConfigs[ci].baud;
        bestConfig  = scanConfigs[ci].config;
        bestRxInv   = (inv == 1);
        bestLabel   = scanConfigs[ci].label;
      }
    }
  }

  if (bestScore > 0) {
    gBaud     = bestBaud;
    gConfig   = bestConfig;
    gRxInvert = bestRxInv;
    gTxInvert = (STOVE_INVERT_TX != 0);
    applyUartParams();
    Serial.print(">>> MEILLEURE config : ");
    Serial.print(bestLabel);
    Serial.print(", rxInv=");
    Serial.print(bestRxInv ? "1" : "0");
    Serial.print(", score=");
    Serial.println(bestScore);
  } else {
    Serial.println(">>> Aucune combinaison ne produit d'octets decodes.");
    Serial.println("    Verifier le câblage opto / que la carte est alimentee.");
    applyUartParams();   // retour a la config macros par defaut
  }
}
// ---------------------------------------------------------------------------
//  Balayage des adresses d'une zone memoire (RAM loc 0x00 / EEPROM loc 0x20)
//  avec la config UART courante. Affiche toute reponse utilisee.
// ---------------------------------------------------------------------------
void stoveScanAddresses(uint8_t location) {
  const char* zone = (location == 0x00) ? "RAM" : "EEPROM";
  Serial.println();
  Serial.print("======= SCAN ADRESSES ");
  Serial.print(zone);
  Serial.print(" (0x00-0x");
  printHex(SCAN_ADDR_MAX);
  Serial.println(") =======");
  Serial.print("config : ");
  printUartParams();
  Serial.println();

  uint32_t hits = 0, cksOk = 0;
  uint8_t raw[8]; uint8_t rl; bool ck;

  for (uint16_t a = 0; a <= SCAN_ADDR_MAX; a++) {
    esp_task_wdt_reset();

    int n = scanProbeOne(location, (uint8_t)a, raw, rl, ck);

    if (rl >= 4) {
      hits++;
      if (ck) cksOk++;

      // Affiche en detail (reponse = raw[2], raw[3])
      Serial.print("  A=0x");
      printHex((uint8_t)a);
      Serial.print("  recu=");
      Serial.print(n);
      Serial.print("  brut=[");
      for (uint8_t k = 0; k < rl; k++) {
        printHex(raw[k]);
        Serial.print(' ');
      }
      Serial.print("]  checksum=");
      Serial.println(ck ? "OK" : "echec");
    }

    if ((a % 32) == 31) {
      Serial.print("  ... progression ");
      Serial.print(((uint16_t)a + 1) * 100 / (SCAN_ADDR_MAX + 1));
      Serial.println(" %");
    }
  }

  Serial.print(">>> ");
  Serial.print(zone);
  Serial.print(" : ");
  Serial.print(hits);
  Serial.print(" adresse(s) avec reponse, dont ");
  Serial.print(cksOk);
  Serial.println(" checksum conforme(s)");
}
// ---------------------------------------------------------------------------
//  Menu serie (jeu interactif)
// ---------------------------------------------------------------------------
void printMenu() {
  Serial.println();
  Serial.println("-------- MENU --------");
  Serial.println("  ?  aide");
  Serial.println("  s  scan complet (configs + adresses RAM/EEPROM)");
  Serial.println("  c  scan des configs uniquement");
  Serial.println("  a  balayage adresses RAM/EEPROM (config courante)");
  Serial.println("  e  test echo (loopback sans poele)");
  Serial.println("  p  mode polling normal");
  Serial.println("  1  1200/8N2      2  1200/8N1");
  Serial.println("  3  4800/8N2      4  4800/8N1");
  Serial.println("  5  9600/8N2      6  9600/8N1");
  Serial.println("  r  toggle inversion RX");
  Serial.println("  t  toggle inversion TX");
  Serial.println("  b  afficher la config UART courante");
  Serial.println("  v  valeurs lues une fois (poll immediat)");
  Serial.println("----------------------");
}

void handleSerialCommands() {
  while (Serial.available()) {
    char ch = (char)Serial.read();

    if (ch == '?') {
      printMenu();
      continue;
    }

    // Commandes de lancement (bloquantes) : on pose scanBusy pendant le scan.
    if (ch == 's') {
      scanBusy = true;
      stoveScanConfigs();
      stoveScanAddresses(0x00);
      stoveScanAddresses(0x20);
      scanBusy = false;
      continue;
    }
    if (ch == 'c') {
      scanBusy = true;
      stoveScanConfigs();
      scanBusy = false;
      continue;
    }
    if (ch == 'a') {
      scanBusy = true;
      stoveScanAddresses(0x00);
      stoveScanAddresses(0x20);
      scanBusy = false;
      continue;
    }
    if (ch == 'e') {
      scanBusy = true;
      echoTest();
      scanBusy = false;
      continue;
    }
    if (ch == 'v') {
      stovePollAll();
      continue;
    }
    if (ch == 'p') {
      Serial.println("Mode polling normal.");
      continue;
    }
    if (ch == 'b') {
      Serial.print("Config UART : ");
      printUartParams();
      Serial.println();
      continue;
    }

    // Changements de config rapides
    bool changed = true;
    switch (ch) {
      case '1': gBaud = 1200; gConfig = SERIAL_8N2; break;
      case '2': gBaud = 1200; gConfig = SERIAL_8N1; break;
      case '3': gBaud = 4800; gConfig = SERIAL_8N2; break;
      case '4': gBaud = 4800; gConfig = SERIAL_8N1; break;
      case '5': gBaud = 9600; gConfig = SERIAL_8N2; break;
      case '6': gBaud = 9600; gConfig = SERIAL_8N1; break;
      case 'r': gRxInvert = !gRxInvert; break;
      case 't': gTxInvert = !gTxInvert; break;
      default: changed = false; break;
    }
    if (changed) {
      applyUartParams();
      Serial.print("Config UART : ");
      printUartParams();
      Serial.println();
    }
  }
}
//  Bloc de debug periodique
// ---------------------------------------------------------------------------
void printDebugBlock(uint32_t now) {
  Serial.println("------------------------------------------------------------");
  Serial.print("[debug] uptime ");
  Serial.print(now / 1000UL);
  Serial.println(" s");

  Serial.print("  octets : ");
  Serial.print(windowBytes);
  Serial.print(" sur cette fenetre | ");
  Serial.print(totalBytes);
  Serial.println(" total");

  // --- Valeurs decodees (mode interrogateur TX) ---
  if (STOVE_USE_TX) {
    Serial.println("  --- valeurs lues ---");
    for (uint8_t i = 0; i < STOVE_READS_COUNT; i++) {
      Serial.print("    ");
      Serial.print(stoveReads[i].label);
      Serial.print(" (");
      printHex(stoveReads[i].location);
      Serial.print("/");
      printHex(stoveReads[i].address);
      Serial.print(") : ");

      if (!stoveValid[i]) {
        Serial.println("pas de reponse");
        continue;
      }

      Serial.print("0x");
      printHex(stoveValues[i]);
      Serial.print(" = ");
      Serial.print(stoveValues[i], DEC);

      // Decodages connus (mapping eni23 / ESPHome)
      if (stoveReads[i].address == 0x21) {
        Serial.print("  (");
        Serial.print(stoveStateName(stoveValues[i]));
        Serial.print(")");
      } else if (stoveReads[i].address == 0x01) {
        Serial.print("°C (valeur/2)");
      } else if (stoveReads[i].address == 0x37) {
        Serial.print(" rpm (x10)");
      }

      // Verification checksum (adresse + valeur == 1er octet)
      bool ck = ((uint8_t)(stoveReads[i].address + stoveValues[i]) ==
                 stoveRaw[i][0]);
      Serial.print(ck ? "  [checksum OK]" : "  [checksum INVALIDE]");

      if (!ck && stoveRawLen[i] > 0) {
        Serial.print("  brut=[");
        for (uint8_t k = 0; k < stoveRawLen[i]; k++) {
          printHex(stoveRaw[i][k]);
          Serial.print(' ');
        }
        Serial.print("]");
      }
      Serial.println();
    }
  }

  Serial.print("  signal : idle=");
  Serial.print(idleLevelHigh ? "HAUT" : "BAS");
  Serial.print(", transitions=");
  Serial.print(diagTransitions);

  if (diagTransitions > 0) {
    Serial.print(", largeur impulsions bas=");
    Serial.print(minLowUs == 0xFFFFFFFF ? 0UL : minLowUs);
    Serial.print("-");
    Serial.print(maxLowUs);
    Serial.print("us haut=");
    Serial.print(minHighUs == 0xFFFFFFFF ? 0UL : minHighUs);
    Serial.print("-");
    Serial.println(maxHighUs);
    Serial.println("us");
  } else {
    Serial.println();
  }

  // --- Interpretation pour le diagnostic ---
  if (STOVE_USE_TX) {
    if (stoveAnyReply) {
      Serial.println("  RESULTAT  : INTERROGATION OK, le poele repond !");
      Serial.println("              -> octets decodes ci-dessus (verifier");
      Serial.println("                 checksum [INVALIDE] pour ajuster adresses).");
    } else if (diagTransitions > 0) {
      Serial.println("  RESULTAT  : notre TX pilote le fil (transitions>0)");
      Serial.println("              mais le poele ne repond pas.");
      Serial.println("              -> verifier le câblage TX opto + la carte est");
      Serial.println("                 alimentee, et les inversions TX/RX.");
      if (DIAGNOSTIC_MODE) {
        printWaveform();
      }
    } else {
      Serial.println("  RESULTAT  : AUCUNE transition => notre TX ne pilote pas");
      Serial.println("              le fil (ou inversion TX a l'envers).");
      Serial.println("              -> verifier câblage TX opto, GPIO21, et le");
      Serial.println("                 tirage de RX (pull-up/down).");
    }
  } else {
    // Mode passif (sniffer) : interprétation de ce qui est capté
    if (windowBytes > 0) {
      Serial.println("  RESULTAT  : SNIFF OK, des octets sont decodes !");
    } else if (diagTransitions > 0) {
      Serial.println("  RESULTAT  : signal PRESENT mais aucun octet decode.");
      Serial.println("              -> cause probable : inversion RX ou baud rate.");
      Serial.print  ("              -> si largeur ~833us : compatible 1200,");
      Serial.println("  tester STOVE_INVERT_RX=1.");
      if (DIAGNOSTIC_MODE) {
        printWaveform();
      }
    } else {
      Serial.println("  RESULTAT  : AUCUN signal sur le fil.");
      Serial.println("              -> sans console/module wifi actif, il n'y a");
      Serial.println("                 aucun trafic spontane (esclave).");
    }
  }

  Serial.print("  config   : UART0 ");
  Serial.print(gBaud);
  Serial.print(" ");
  Serial.print((gConfig == SERIAL_8N1) ? "8N1" : (gConfig == SERIAL_8N2) ? "8N2" : "?");
  Serial.print(" | RX=");
  Serial.print(gRxInvert ? "INV" : "normal");
  Serial.print(",");
  Serial.print(rxPullStr);
  Serial.print(" GPIO20 | TX=");
  Serial.print(gTxInvert ? "INV" : "normal");
  Serial.print(STOVE_USE_TX ? " ACTIF (GPIO21)" : " reserve");
  Serial.println();
  Serial.println("  Moniteur  : Serial USB 115200");
  Serial.println("------------------------------------------------------------");

  // Reset des compteurs de fenetre
  windowBytes = 0;
#if !STOVE_USE_TX
  windowFrames = 0;
#endif
  diagTransitions = 0;
  minHighUs = 0xFFFFFFFF; maxHighUs = 0;
  minLowUs  = 0xFFFFFFFF; maxLowUs  = 0;
  lastLevel = digitalRead(PIN_STOVE_RX);
  lastChangeUs = micros();
}

// ---------------------------------------------------------------------------
//  setup / loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("===========================================");
  Serial.println(STOVE_USE_TX
                 ? "  MICRONOVA - mode INTERROGATEUR (TX)"
                 : "  MICRONOVA - mode SNIFFER passif");
  Serial.println("===========================================");

  // UART0 matériel sur GPIO20 (RX) / GPIO21 (TX).
  // La config (baud, format, inversions, pull) est pilotee par les variables
  // runtime gBaud/gConfig/gRxInvert/gTxInvert, initalisees depuis les macros.
  gBaud     = STOVE_BAUD;
  gConfig   = STOVE_CONFIG;
  gTxInvert = (STOVE_INVERT_TX != 0);
  gRxInvert = (STOVE_INVERT_RX != 0);
  applyUartParams();

  // Test du niveau de repos de la ligne (lecture physique du pad)
  detectIdleLevel();

  Serial.print("  Idle niveau GPIO20 : ");
  Serial.println(idleLevelHigh ? "HAUT (normal)" : "BAS (anormal)");
  Serial.print("  Config UART        : ");
  printUartParams();
  Serial.println();
  Serial.println();

  if (STOVE_USE_TX) {
    Serial.println("  L'ESP interroge le poele (toutes les 5 s en mode polling).");
    Serial.println("  IMPORTANT : debrancher le module wifi / console filaire");
    Serial.println("  pendant les tests (2 maitres = collisions).");
  } else {
    Serial.println("  Mode passif : aucune emission, ecoute seule.");
  }
  Serial.println("  Tapez '?' pour afficher le menu serie.");
  Serial.println();

  lastLevel = digitalRead(PIN_STOVE_RX);
  lastChangeUs = micros();
  lastDebugMs = millis();

  // Scan automatique au demarrage, si configure.
  if (STOVE_SCAN_MODE && STOVE_USE_TX) {
    scanBusy = true;
    stoveScanConfigs();
    stoveScanAddresses(0x00);
    stoveScanAddresses(0x20);
    scanBusy = false;
    Serial.print("Config UART retenue : ");
    printUartParams();
    Serial.println();
  }
}

void loop() {
  uint32_t now = millis();

  // --- Diagnostic : comptage des transitions du signal sur GPIO20 ---
  updateDiag(micros());

  // --- Commandes serie (menu interactif) ---
  handleSerialCommands();

  // --- Cycle de debug periodique (interrogation + affichage) ---
  // Pendant un scan bloque, on saute le polling.
  if (!scanBusy && (now - lastDebugMs >= DEBUG_INTERVAL_MS)) {
    if (STOVE_USE_TX && ECHO_TEST) {
      echoTest();
    } else if (STOVE_USE_TX) {
      stovePollAll();
    }
    printDebugBlock(now);
    lastDebugMs = now;
  }
}