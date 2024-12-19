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

![Ablauf-V2 drawio](https://github.com/user-attachments/assets/e44a87c0-f1eb-4fd6-8971-e4bf91ab9b20)



