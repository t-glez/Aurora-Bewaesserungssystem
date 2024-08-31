# Aurora-Bewaesserungssystem
 ROS and Arduino programms for the Aurora-Bewaesserungssystem
## To-Do

## ROS package

### Kommunikation
![image](https://github.com/user-attachments/assets/aab795fb-7823-4ad8-b5b5-bc5dc221ffc1)

Gelöst mit einem einzigen Arduino und über eine Action und nicht einzelne Topics. Message kann wie folgt aussehen:

Goal  
string  seite #Rechts oder links  
int pos #Position des Baumes

#Published result  
Bool goal_achieved #Erfolgreiche Bewässerung

#Feedback message  
int current_pos #Aktuelle position der Motoren  
int current_state #Aktuelle Schritt der Schrittkette  
Zusätzlich wird ein Topic für den Wasserstand gebraucht


## Arduino

### Process

![image](https://github.com/user-attachments/assets/442106db-0deb-496b-b3c5-eff94e6a588e)

Das Diagramm gilt erstmal nur für eine Seite. Es muss eine Verzweigung bei Übergang 1 hinzugefügt werden, um die andere Seite zu bedienen. Außerdem muss es eine Möglichkeit geben, den Prozess abbrechen zu können. Das heißt in Schritt 40 Springen zu können.

Bei jedem Schrittwechsel soll dies als Feedback angezeigt werden.


