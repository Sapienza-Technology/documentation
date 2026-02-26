# Arm code repository

Per la tua documentazione, puoi riassumere il funzionamento così:  
1. Lancio: ros2 launch bracc8_control maintenance_task.launch.py avvia tutto.  
2. Inizializzazione: interactive_control_node carica la cinematica e avvia il controller. realsense_node inizia a streammare dati.  
3. Percezione: maintenance_task_node usa ArUco per definire il piano di lavoro (panel_frame) e poi attiva YOLO per trovare gli interruttori.  
4. Pianificazione: Le coordinate 3D degli oggetti vengono passate al trajectory_generator, che calcola l'IK e genera percorsi fluidi.  
5. Controllo: bracc8_controller esegue la traiettoria, inviando comandi al robot e monitorando la sicurezza.  
6. Visualizzazione: L'operatore supervisiona tutto su RViz, vedendo sia il robot reale che l'anteprima delle intenzioni del robot (robot fantasma e linee di percorso).

## bracc8_interfaces
1. Ruolo: Definisce tutte le interfacce di comunicazione (messaggi e servizi) per l'intero workspace del braccio robotico.
2. Messaggi Principali:  
   1. ControlCommandMsg: Messaggio omnicomprensivo per inviare comandi di teleoperazione (movimento, cambio modalità, registrazione traiettorie) da un dispositivo di input al robot.  
   2. TeleopStateMsg: Messaggio di stato che descrive la modalità di controllo attualmente attiva.  
   3. DetectionArray: Messaggio per la pubblicazione di risultati da un sistema di visione artificiale.  
3. Servizi Principali:
    1. MaintenanceDetectTag e MaintenanceNextPose:
    Servizi per orchestrare un task di manutenzione guidata passo-passo, combinando percezione e movimento.  

## bracc8_description
1. Scopo: Fornisce la descrizione URDF del robot, includendo geometria visiva, volumi di collisione e proprietà inerziali per la simulazione e il controllo.
2. Cinematica:
a. Robot antropomorfo a 6 gradi di libertà (6DOF).
b. Giunti definiti come continuous (rotazione continua), con limiti software impostati a ±π radianti.
3. End-Effector e Sensori:
a. Il link ee rappresenta il Tool Center Point (TCP).
b. Configurazione Eye-in-Hand: La telecamera Intel RealSense è montata solidalmente all'end-effector (offset: x=3cm, z=3cm).
4. File Mesh: Le geometrie 3D reali sono caricate dalla cartella meshes/ per una visualizzazione realistica in RViz.

## bracc8_control

### bracc8_control/bracc8_control
This folder contains the main system logic. It is organized in modules to separate responibilities. 

Folder layout

    common/                         # Global definitions to avoid redundancy
        constants.py                # 
    communication/                  # Handles ROS2 inteface, isolating control logic from middleware
        ros_communicator.py         #
    kinematics/                     # Kinematic calculations for forward and inverse kinematics
        kinematics.py               #
    trajectory_generation/          # Path planning logic
        trajectory_generator.py     #
    control/                        # Business logic and state handling
        bracc8_controller.py        #
    input_handling/                 # Input hardware abstraction
        getch.py                    # Utility  to read a single char from keyboard on Linux without pressing Enter (raw input).
        input_mappings.py           #
        input_handler.py            # Handles joystick and keyboard logic
    nodes/                          # Executable ROS2 nodes
        camera_tf_publisher.py
        teleop_node.py
        interactive_control_node.py
        maintenance_task_node.py
        semaforo_node.py    

=== "📂 common"
    #### constants.py
    * Definisce BRACC8_JOINT_NAMES, un array numpy con le stringhe 'joint1'...'joint6'.
    * Garantisce che tutti i nodi (ROS communicator, TF publisher, ecc.) usino gli stessi identici nomi definiti nell'URDF.  
=== "📂 communication"
    Gestisce l'interfaccia con ROS2, isolando la logica di controllo dal middleware.
    #### ros_communicator.py
      1. QoS (Quality of Service):
        * Configura profili diversi: Reliable per i comandi critici (es.: traiettorie, stop emergenza) e Best Effort per dati ad alta frequenza (es. stato giunti).  
      2. Publisher:  
        1. /command/joint_trajectory_point:
           * Invia posizioni/velocità target al driver del braccio.
        2. /rviz/commanded/trajectory_preview:
           * Invia un messaggio Path per visualizzare la linea verde in RViz prima del movimento.
      3. Subscriber:
         1. /state/joint_states:
            1. Riceve il feedback reale dal robot.
      4. Servizi:
         1. Gestisce chiamate asincrone (call_async) per sensori esterni (cella di carico, pH) per non bloccare il loop di controllo. 
=== "📂 kinematics"
    Il motore matematico del robot. Implementa la geometria e i calcoli differenziali.
    #### kinematics.py:
    Classe Kinematics: Definisce la tabella DH (Denavit-Hartenberg) delbraccio.
    1. forward_kinematics(q):
    a. Calcola la posa Cartesiana (matrice 4x4 SE3) data la
    configurazione dei giunti.
    2. analytical_inverse_kinematics(T):
    a. Implementa una soluzione geometrica chiusa
    (disaccoppiamento polso sferico). È il metodo preferito
    perché veloce ed esatto. Restituisce tutte le possibili
    soluzioni (es. gomito alto/basso).
    3. numeric_inverse_kinematics(T):
    a. Solutore iterativo (Levenberg-Marquardt) usato come
    fallback se la soluzione analitica fallisce.
    4. differential_inverse_kinematics(q, v):
    a. Usato per il controllo joystick. Calcola le velocità dei giunti
    (q˙) necessarie per ottenere una velocità cartesiana (v)
    usando lo Jacobiano pseudo-inverso con smorzamento
    (DLS) per gestire le singolarità. Include un algoritmo di
    Velocity Scaling per rallentare dolcemente vicino ai
    finecorsa.
    5. get_camera_pose(q):
    a. Calcola la posizione della telecamera (Eye-in-Hand)
    componendo le trasformate cinematiche.  
=== "📂 trajectory_generation"
    Pianificatore di percorsi (Path Planner).
    #### trajectory_generator.py:
    Classe TrajecotryGenerator:
    1. generate_joint_trajectory:
    a. Genera traiettorie fluide nello spazio dei giunti usando
    polinomi di 5° grado (jtraj o mstraj di robotics toolbox).
    Garantisce continuità in velocità e accelerazione.
    1. generate_cartesian_trajectory:
    a. Prende una lista di Waypoint Cartesiani. Per ogni punto,
    calcola l'IK scegliendo la soluzione più vicina alla
    configurazione precedente (per evitare scatti bruschi). Poi
    interpola nello spazio dei giunti.
    1. generate_incremental_rotation:
    a. Funzioni specifiche per la teleoperazione. Calcolano
    micro-traiettorie per piccoli spostamenti lineari o rotazioni
    rispetto al frame Base o Tool.
    1. get_predefined_trajectory:
    a. Contiene movimenti hardcoded (es. "home", "paletta")
    utili per test e demo.  
=== "📂 control"
    La logica di business e gestione dello stato.
    #### bracc8_controller.py:
    Classe : Mantiene lo stato del robot (current_q, control_mode,
    emergency_stop).
    1. process_command:
    a. Il "cervello" reattivo. Riceve un ControlCommand e
    decide quale azione intraprendere (es. calcolare
    cinematica inversa, avviare una traiettoria, cambiare
    frame).
    1. Gestione Thread:
    a. Metodo execute_trajectory_non_blocking. Lancia
    l'esecuzione fisica su un thread separato per permettere
    all'utente di interrompere il movimento (Stop Emergenza)
    in qualsiasi momento.
    1. Recorder:
    a. Metodi _save_all_trajectories e _load_... per gestire la
    persistenza dei waypoint su file JSON.  
=== "📂 input_handling"
    Astrazione dell'hardware di input.
    #### getch.py: Utility per leggere un singolo carattere da tastiera su Linux
    senza premere Invio (raw input).
    #### input_mappings.py:
    a. Definisce Enum: ControlCommand (comandi logici),
    ControlMode (stati operativi),
    ReferenceFrame.JoystickMappings: Dizionario che mappa indici
    fisici (es. Axis 1) a nomi logici (es. 'LS_Y'). Permette di
    supportare controller diversi (Xbox, PS4) cambiando solo questo
    file.
    #### input_handler.py:
    a. Classe : Gestisce pygame (per joystick) e tastiera.
    b. Logica Joystick: Applica deadzone (zone morte) e converte le
    letture degli assi analogici in un vettore velocità 6D
    normalizzato.  
=== "📂 nodes"
    I nodi eseguibili ROS2.
    #### camera_tf_publisher.py:
    a. Si sottoscrive a /joint_states.
    b. Usa kinematics.get_camera_pose per calcolare dove si trova la
    telecamera rispetto alla base.
    c. Pubblica la trasformata dinamica su /tf. Fondamentale per la
    configurazione Eye-in-Hand.
    #### teleop_node.py:
    a. Nodo "leggero" che gira a 20Hz.
    b. Interroga InputHandler e pubblica ControlCommandMsg.
    c. Ascolta TeleopStateMsg per adattare i comandi allo stato attuale
    del robot.
    #### interactive_control_node.py:
    a. Il nodo centrale. Istanzia il Bracc8Controller e collega i callback
    ROS. È il ponte tra il mondo ROS (messaggi) e la logica Python
    pura (Controller).
    1. maintenance_task_node.py:
    a. Nodo autonomo complesso per il task di manutenzione.
    b. Implementa una Macchina a Stati Finiti (FSM):
    i. DISCOVERING: Trova ArUco.
    ii. LOCALIZING: Costruisce panel_frame.
    iii. SCENE_UNDERSTANDING: Usa YOLO + Depth per
    mappare gli oggetti 3D.
    iv. EXECUTE: Pianifica ed esegue l'azione (es. flip switch).
    c. Visione 3D: Metodo get_object_pose trasforma pixel 2D ->
    punto 3D Camera -> punto 3D Panel Frame usando TF buffer.
    #### semaforo_node.py (Utility):
    a. Un semplice subscriber per controllare LED esterni
    (probabilmente per debug stato sistema).

### bracc8_control/launch - Configurazione Avvio
Script Python per l'orchestrazione dei nodi.
1. bracc8_visualization.launch.py:
a. Lancia l'infrastruttura base.
b. Key Feature:
i. Avvia due robot_state_publisher.
c. Uno nel namespace rviz/commanded (robot fantasma/target) e
uno in rviz/real (robot fisico/feedback). Questo permette di
visualizzare l'errore di inseguimento.Carica l'URDF via xacro.
d. Lancia RViz con la configurazione salvata.
2. realsense.launch.py:
a. Avvia il driver realsense2_camera_node.
b. Abilita align_depth (allineamento pixel colore-profondità) e
pointcloud.Lancia anche aruco_detector per la localizzazione
pannello.
3. maintenance_task.launch.py:
a. Launch file "master" per la demo autonoma.
b. Include bracc8_visualization e realsense.
c. Avvia yolo_detector_lifecycle (il nodo AI) passando il percorso
del modello .pt.
d. Avvia maintenance_task_node con i parametri operativi (offset
di approccio, distanze flip).
4. debug_aruco/yolo_realsense.launch.py:
a. Launch file leggeri per testare sottosistemi (solo ArUco o solo
YOLO) senza avviare l'intero stack del robot.
5. jetson.launch.py:
a. Probabile variante ottimizzata per girare su hardware NVIDIA
Jetson (potrebbe avere configurazioni specifiche per le risorse
limitate).

### bracc8_control/models - Risorse AI
Contiene i file binari delle reti neurali.
1. best.pt, best_11n_newseg.pt :
a. Sono pesi pre-addestrati per il modello YOLOv8 (You Only Look
Once).
b. Contengono la "conoscenza" necessaria per identificare
interruttori, prese e leve nelle immagini RGB. Vengono caricati
dal nodo yolo_detector.

### bracc8_control/rviz - Configurazione Grafica
1. bracc8_view.rviz:
a. File YAML salvato da RViz
b. .Configura i Display attivi:
i. RobotModel (Commanded): Alpha 0.3 (trasparente).
ii. RobotModel (Real): Alpha 1.0 (solido).
iii. PointCloud2: Visualizza i dati RGBD della RealSense.
iv. Path: Visualizza il topic
/rviz/commanded/trajectory_preview (linea verde).
c. Salva la posa della telecamera virtuale per avere subito la
visuale corretta all'avvio.

### bracc8_control/data - Persistenza Dati
1. recorded_trajectories.json:
a. Database JSON dove vengono salvate le traiettorie registrate
dall'utente.
b. Struttura: Nome Traiettoria -> Lista di Waypoint -> (Posa, Stato
GripperConfigurazione Giunti).
c. Permette la riesecuzione ("Play") di movimenti complessi
insegnati manualmente.

### bracc8_control/test - Validazione Matematica
Questa cartella è molto interessante perché mostra il processo di ingegneria dietro il
codice Python. Contiene script MATLAB usati per prototipare e validare gli algoritmi.
1. compute_A_matrices.m: Calcolo simbolico delle matrici di
trasformazione omogenea dalla tabella
2. DH.DH_to_JA /DH_to_JL : Calcolo simbolico dello Jacobiano Analitico
e Lineare. Serve a verificare che lo Jacobiano calcolato in Python sia
corretto.
3. cubic_poly_coeff.m: Validazione matematica della generazione di
traiettorie
4. cubiche.newton_method.m: Implementazione di prova dell'algoritmo di
Newton per la cinematica inversa numerica.
5. find_singularity.m: Analisi simbolica del determinante dello Jacobiano
per trovare le configurazioni singolari (dove il robot perde gradi di
libertà).