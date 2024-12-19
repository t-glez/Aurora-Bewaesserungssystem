# Aurora-Bewaesserungssystem
 ROS and Arduino programms for the Aurora-Bewaesserungssystem

### Kommunikation

Um Kommunikation mit dem Controllino zu starten muss folgendes gestartet werden:

```
rosrun rosserial_python serial_node.py tcp 11411
```

Um Prozess zu starten muss ein std_msgs/String "data: 'Links'" oder "data: 'Rechts'" geschickt werden.

Cancell mit 


## Arduino

### Process

![image](https://github.com/user-attachments/assets/442106db-0deb-496b-b3c5-eff94e6a588e)

Das Diagramm gilt erstmal nur für eine Seite. Es muss eine Verzweigung bei Übergang 1 hinzugefügt werden, um die andere Seite zu bedienen. Außerdem muss es eine Möglichkeit geben, den Prozess abbrechen zu können. Das heißt in Schritt 40 Springen zu können.

Bei jedem Schrittwechsel soll dies als Feedback angezeigt werden.


