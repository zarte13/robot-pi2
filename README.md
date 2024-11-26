# robot-pi2

Guide d'installation:
1. Installer Arduino IDE
2. Installer board manager "ESP32" by Espressif Systems
3. Installer les library Arduino suivante: ESP32Servo et PS4Controller
4. Installer l'outil [SixaxisPairTool](https://sixaxispairtool.en.lo4d.com/windows)
5. Cloner ce repo
6. Ouvrir le fichier "GetMacAdress" dans Arduino IDE
7. Chosir le board: Tools -> Board: xxx -> esp32 -> DOIT ESP32 DEVKIT V1
8. Uploader le code, puis garder en note l'addresse MAC
9. Connecter la manette à l'ordinateur, puis executer SixaxisPairTool, puis insérer l'addresse MAC
10. Ouvrier le fichier "code_robot" dans Arduino IDE
11. Uploader le code au ESP32

Guide d'utilisation: 
1. Alimenter le robot
2. Appuyer pendant 1 sec sur le button central de la manette choisi pendant l'installation
3. Attendre que la lumière derrière la manette devienne d'une couleur sans flasher
4. Utiliser les joystick et boutton R1, L1 et R2, L2 pour controller le robot
