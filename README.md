# Smartsnap_1
Librairies utilisées :
  -	SPI.h : Librairie pour la communication SPI
  -	SdFat.h : plus rapide que la librairie SD.h
  -	Adafruit_FXAS21002C.h : Gyroscope du NXP Precision 9-DOF
  -	Adafruit_FXOS8700.h : Accéléromètre et magnétomètre du NXP Precision 9-DOF
  -	Adafruit_Sensor.h : Librairie pour les capteurs en I2C
  -	Wire.h : Librairie pour la communication I2C
  -	RTCZero.h : Librairie pour le décompte du temps
  -	MS5837.h : Librairie pour le capteur de pression/température
  -	TimeLib.h : Libraire pour avoir la date et l’heure de programmation de la carte
  -	ArduinoLowPower.h : Librairie pour le Sleep Mode du microcontrôleur SAMD21

Fonctionnalités : 
  -	La carte passe en Sleep Mode une fois qu’elle a été programmée.
  
  -	Réveil de la carte lorsque la pression dépasse 2 bars ou que le câble USB est branché.
  -	Câble USB branché : Accès aux fichiers stockés sur la carte SD via un terminal.
  -	Pression supérieure à 2 bars : Création d'un fichier dans lequel sont écris les données.
  
  - Lorsqu’aucune de ces conditions ne sont pas remplis : Fermeture des fichiers et endormissement de la carte pour 1 minute.
  -	Sauvegarde de la batterie toutes les 30 minutes.
