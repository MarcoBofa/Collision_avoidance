# Collision_avoidance
Algoritmo per la prevenzione dinamica della collisioni con l'utilizzo di Probabilistic RoadMaps e Rapidly-Exploring Random Trees

<br/>
<br/>

## Documentazione

Il codice è stato documentato utilizzando doxygen. La documentazione è consultabile aprendo il file **index.html** che si trova in:

> collision_avoidance -> doxygen_doc -> html -> doc -> index.html

<br/>
<br/>


## Istruzioni per l'installazione e utilizzo del codice

- Per lo sviluppo del codice è stato utilizzato **Ubuntu 20.04.02** con **ROS Noetic**.
<br/>

1. Se ROS non è già installato seguire la procedura descritta nel dettaglio in [Ros Noetic installation](http://wiki.ros.org/noetic/Installation/Ubuntu)

<br/>
2. Una volta installato ROS noetic si procede l'Installazione di MoveIt seguendo il tutorial mostrato in  [MoveIt Installation](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html) **FACENDO ATTENZIONE PER0'** al passaggio finale di configurazione di eseguire il comando: 

```
catkin_make    
```
dopo il comando:
```
 catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
```
INVECE DI  **catkin build**
<br/>


3. Una volta creato e compilato il workspace il prossimo step è da svolgere non da terminale ed è quello di copiare le 3 cartelle (soft_robotics, collision_avoidance e fmauch_universal_robot) presenti nella cartella ROS_packeges situata in Thesis_files e incollarle all'interno della cartella src presente nel workspace creato. ATTENZIONE A COPIARE LE SINGOLE CARTELLE E NON L'INTERA CARTELLA CHIAMATA ROS_packages.

- per controllare se tutto è configurato in maniera corretta:
  
```
cd ~/catkin_ws/src
ls 
```
dovrebbe dare il seguente output:
<br/>
<img width="647" alt="Schermata 2022-09-08 alle 10 49 45" src="https://user-images.githubusercontent.com/105377367/189078871-f4bf9833-2c92-44b1-8578-252b6dbdb0e3.png">
<br/>

- A questo punto si può concludere compilando nuovamente con 
 
```
catkin_make
```

- Potrebbe essere necessaria l'installazione di ulteriori pacchetti da installare eseguendo in cd ~/catkin_ws/ il comando:
```
rosdep install --from-paths src --ignore-src -r -y
```

Nel caso di errori di compilazione spegnere il computer (o macchina virtuale) e riprovare.


<br/>


4. Una volta compilato tutto è possibile riprodurre le simulazioni. 

- Sono presenti 3 Test, per cambiare il tipo di test da eseguire bisogna andare nel pacchetto
> collision_avoidance -> src -> launch -> collision_av.launch

commentare e scommentare la simulazione che si vuole eseguire.
Lasciandone **SOLO UNA** scommentata.
<br/>
<br/>

- A questo punto si possono aprire due finestre del terminale ed eseguire nella prima 

```
roslaunch ur5e_gripper_moveit_config ur5e_gazebo.launch
```

Che avvia gazebo ( nel quale purtroppo sarà presente solo il robot perchè i modelli (tavolo, ostacoli etc.) vengono salvati in locale in .gazebo, anche se rappresenta solo un aspetto di visualizzazione visto che i controlli delle collisioni e degli ostacoli avvengono su Rviz). 
Viene avviato inoltre Rviz in cui dovrebbe subito dopo comparire il tavolo di laboratorio, mentre il robot dovrebbe iniziare il movimento verso la posizione di "riposo".
<br/>
<br/>

- Nella seconda finestra eseguire il comando:
```
roslaunch collision_avoidance collion_av.launch
```

Per prima cosa verrà richiesto a terminale all'utente di inserire il numero di test selezionato (quello scommentato come descritto nella parte precedente), inserito il numero la simulazione comincerà e il robot inizierà a muoversi. 
<br/>
<br/>

Se dovesse succedere che il programma dia errori o si blocchi (non dovrebbe ma con tutti i requisiti di installazione necessari non si sa mai) usare Ctrl^c per fermare la simulazione e riavviarla con i comandi mostrati.
