# Passerelle HomeKit-CAN pour Portail

Intégration d'un portail motorisé avec Apple HomeKit via bus CAN et ESP32.

## 📋 Description

Ce projet implémente une passerelle bidirectionnelle entre Apple HomeKit et un portail motorisé communicant via bus CAN. Il permet de :

- **Commander le portail** depuis l'app Maison (iPhone/iPad/Mac)
- **Visualiser l'état en temps réel** (Fermé, Ouverture, Ouvert, Fermeture, Obstacle)
- **Synchroniser automatiquement** l'interface HomeKit avec l'état réel du portail, même en cas de commande par télécommande externe
- **Détecter les obstacles** et les signaler dans HomeKit

## ✨ Caractéristiques techniques

- **Architecture multi-cœur** : HomeSpan sur Core 0, CAN sur Core 1 (FreeRTOS)
- **Communication thread-safe** avec mutex
- **Timeout de surveillance** du bus CAN (15s)
- **Support OTA** (mise à jour sans fil)
- **Indicateurs LED** visuels (ouverture/fermeture)
- **Mode debug** configurable

## 🔧 Matériel requis

### Composants principaux
- **ESP32** (Dev Kit v1 ou équivalent)
- **Transceiver CAN** : MCP2562 ou équivalent (5V tolérant)
- **LEDs** (optionnel) : 2 LEDs + résistances pour indication visuelle
- **Alimentation** : 5V pour ESP32 et transceiver CAN

### Schéma de connexion

```
ESP32              MCP2562 (CAN Transceiver)
─────              ──────────────────────────
GPIO 23 ────────► Pin 1 (TXD)
GPIO 22 ◄──────── Pin 4 (RXD)
3.3V ───────────► Pin 3 (VDD)
GND ────────────► Pin 2 (VSS/GND)
                  Pin 6 (CANL) ──┐
                  Pin 7 (CANH) ──┤─► Bus CAN (vers portail)
                                  │
                  Résistance 120Ω (si terminaison nécessaire)

GPIO 16 ────────► LED Rouge (Fermeture) + Résistance
GPIO 17 ────────► LED Verte (Ouverture) + Résistance
```

### Configuration du bus CAN

- **Vitesse** : 125 kbps (configurable via `CAN_BITRATE`)
- **Format** : CAN 2.0A (identifiants 11 bits)
- **Terminaison** : 120Ω si extrémité de bus

## 📦 Bibliothèques nécessaires

Installez via le gestionnaire de bibliothèques Arduino IDE :

1. **HomeSpan** (version 1.8.0 ou supérieure)
   ```
   https://github.com/HomeSpan/HomeSpan
   ```

2. **ACAN_ESP32** (bibliothèque CAN pour ESP32)
   ```
   https://github.com/pierremolinaro/acan-esp32
   ```

## 🚀 Installation

### 1. Configuration Arduino IDE

- Carte : **ESP32 Dev Module**
- Partition Scheme : **Default 4MB with spiffs**
- Flash Frequency : **80MHz**
- Upload Speed : **921600**

### 2. Configuration WiFi et HomeKit

Au premier démarrage, l'ESP32 crée un point d'accès WiFi :

1. Connectez-vous au réseau **HomeSpan-Setup**
2. Ouvrez votre navigateur sur `192.168.4.1`
3. Configurez vos identifiants WiFi
4. Notez le **code de configuration HomeKit** affiché dans le moniteur série

### 3. Ajout dans l'app Maison

1. Ouvrez l'app **Maison** sur votre iPhone
2. Touchez **+** puis **Ajouter un accessoire**
3. Scannez le code QR ou entrez le code à 8 chiffres
4. Suivez les instructions à l'écran

## 🔌 Protocole CAN

### Messages émis par la passerelle (commandes)

| ID CAN  | Description       | Data[0] |
|---------|-------------------|---------|
| `0x200` | Commande OUVRIR   | `0x01`  |
| `0x201` | Commande FERMER   | `0x01`  |

### Messages reçus du portail (états)

| ID CAN  | Description | Data[0] | État HomeKit |
|---------|-------------|---------|--------------|
| `0x100` | FERMÉ       | `0x01`  | CLOSED       |
| `0x100` | OUVRANT     | `0x02`  | OPENING      |
| `0x100` | OUVERT      | `0x00`  | OPEN         |
| `0x100` | FERMANT     | `0x03`  | CLOSING      |
| `0x100` | OBSTACLE    | `0x04`  | STOPPED      |

> **Note** : Ces identifiants sont configurables dans le code (section `IDENTIFIANTS CAN`)

## ⚙️ Configuration personnalisée

### Modifier les identifiants CAN

```cpp
// Messages envoyés (commandes)
#define CAN_ID_CMD_OUVRIR      0x200  // Modifiez selon votre système
#define CAN_ID_CMD_FERMER      0x201

// Messages reçus (états)
#define CAN_ID_ETAT_PORTAIL    0x100
```

### Modifier les états CAN

```cpp
// Valeurs dans data[0] des messages d'état
#define ETAT_OUVERT     0x00  // Adaptez à votre protocole
#define ETAT_FERME      0x01
#define ETAT_OUVRANT    0x02
#define ETAT_FERMANT    0x03
#define ETAT_OBSTACLE   0x04
```

### Activer le mode debug

```cpp
#define DEBUG_MODE true  // Affiche les logs détaillés CAN
```

### Modifier les GPIO

```cpp
static const gpio_num_t CAN_TX_PIN = GPIO_NUM_23;
static const gpio_num_t CAN_RX_PIN = GPIO_NUM_22;
static const gpio_num_t LED_CLOSE_PIN = GPIO_NUM_16;
static const gpio_num_t LED_OPEN_PIN = GPIO_NUM_17;
```

## 🐛 Dépannage

### Le portail ne répond pas aux commandes

1. Vérifiez le câblage CAN (CANH/CANL)
2. Vérifiez la vitesse du bus (`CAN_BITRATE`)
3. Activez `DEBUG_MODE true` pour voir les trames CAN
4. Vérifiez les identifiants CAN de votre système

### Moniteur série affiche "TIMEOUT: Aucun message CAN depuis 15s"

- Le portail n'envoie pas d'états périodiques
- Câblage CAN défectueux
- Transceiver CAN non alimenté ou défaillant
- Terminaison CAN manquante (120Ω)

### HomeKit affiche "Pas de réponse"

- Vérifiez la connexion WiFi de l'ESP32
- Redémarrez le routeur et l'ESP32
- Supprimez et ré-ajoutez l'accessoire dans l'app Maison

### Le switch HomeKit ne se synchronise pas

Ce problème a été corrigé dans la version 0.0.4. Vérifiez que vous utilisez la dernière version du code qui met à jour **à la fois** `currentState` et `targetState`.

## 📊 Surveillance et logs

### Moniteur série (115200 bauds)

Le système affiche :
- État de connexion HomeKit
- Messages CAN émis et reçus (si `DEBUG_MODE`)
- Changements d'état du portail
- Détection d'obstacles
- Timeouts de communication

Exemple de sortie :
```
┌── ETAT CAN RECU ───┐
  Etat portail: OUVERT
  → HomeKit CurrentState: 2 → 0
  → HomeKit TargetState: 1 → 0
```

## 🔐 Sécurité

- ⚠️ **Ne pas exposer** l'ESP32 directement sur Internet
- Utilisez HomeKit via le **hub domestique** Apple (iPad/HomePod/Apple TV)
- Le protocole HomeKit est **chiffré de bout en bout**
- Changez le code de configuration si compromis

## 📈 Évolutions possibles

- [ ] Ajout d'un capteur de température/humidité
- [ ] Historique des ouvertures/fermetures
- [ ] Notifications push en cas d'obstacle
- [ ] Interface web de configuration
- [ ] Support de plusieurs portails
- [ ] Intégration avec alarme maison

## 🤝 Contribution

Ce projet est partagé pour la communauté. N'hésitez pas à :
- Signaler des bugs via Issues
- Proposer des améliorations via Pull Requests
- Partager vos adaptations

## 📄 Licence

Projet open-source libre d'utilisation et de modification.

## ✍️ Auteur

**Dominique**  
Contact : dominique@locoduino.org  
Site : [Locoduino.org](https://www.locoduino.org)

## 🙏 Remerciements

- [HomeSpan](https://github.com/HomeSpan/HomeSpan) de Gregg E. Berman
- [ACAN_ESP32](https://github.com/pierremolinaro/acan-esp32) de Pierre Molinaro
- La communauté Arduino et ESP32

## 📚 Ressources

- [Documentation HomeSpan](https://github.com/HomeSpan/HomeSpan/blob/master/docs/Reference.md)
- [Spécifications CAN 2.0](https://www.can-cia.org/can-knowledge/can/can-specifications/)
- [HomeKit Accessory Protocol](https://developer.apple.com/homekit/)

---

**Version** : 0.0.4  
**Date** : Octobre 2025  
**Testé avec** : ESP32 DevKit v1, HomeSpan 1.8.0, iOS 17+