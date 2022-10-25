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
  
  -	Câble USB branché :
-	              Led rouge allumée
    	Fermeture du fichier sur la carte SD au cas où
    	Ouverture de l’IHM
    	Ouvrir le Tera Term
    	Débrancher puis rebrancher l’USB
    	Accès aux fichiers
      •	Lecture simple des « .txt »
      •	Décodage des « .dat »
    	Faire « 0 » pour quitter l’IHM.
    	Débrancher l’USB
    	Led rouge éteinte
  o	Pression supérieure à 2 bars :
    	Enregistrement du début de l’enregistrement
    	Création d’un fichier « .dat »
    	Sauvegarde de la batterie
    	Réinitialisation des capteurs et de la carte SD
    	Ecriture en format compressé des données
  o	Lorsqu’aucune de ces conditions ne sont pas remplis :
    	Enregistrement de la fin de l’enregistrement
    	Fermeture du fichier sur la carte SD au cas où
    	Sauvegarde de la batterie
    	Mise en place de l’alarme dans 1 minute
    	Endormissement de la carte
  -	Sauvegarde de la batterie toutes les 30 minutes.
