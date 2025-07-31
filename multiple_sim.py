#!/usr/bin/env python3

import os, subprocess, time, rospy, signal
from std_msgs.msg import Bool
import dynamic_reconfigure.client


# === CONFIGURAZIONE ===
UAV_NAME = "uav1"
TMUX_SESSION_NAME = "simulation"

terminal_proc = None
counter = 0

param_sets_1 = [
    {"kp_sliding": 5.0,  "kd_sliding": 5.0 },
    {"kp_sliding": 5.0,  "kd_sliding": 7.5 },
    {"kp_sliding": 5.0,  "kd_sliding": 10.0},
    {"kp_sliding": 7.5,  "kd_sliding": 5.0 },
    {"kp_sliding": 7.5,  "kd_sliding": 7.5 },
    {"kp_sliding": 7.5,  "kd_sliding": 10.0},
    {"kp_sliding": 10.0, "kd_sliding": 5.0 },
    {"kp_sliding": 10.0, "kd_sliding": 7.5 },
    {"kp_sliding": 10.0, "kd_sliding": 10.0},
    {"kp_sliding": 12.5, "kd_sliding": 5.0 },
    {"kp_sliding": 12.5, "kd_sliding": 7.5 },
    {"kp_sliding": 12.5, "kd_sliding": 10.0}

]

param_sets = [
    { "sliding_vel": 2.0, "admittance_vel": 1.0 },
    { "sliding_vel": 2.0, "admittance_vel": 1.5 },
    { "sliding_vel": 2.0, "admittance_vel": 2.0 },
    { "sliding_vel": 2.5, "admittance_vel": 1.0 },
    { "sliding_vel": 2.5, "admittance_vel": 1.5 },
    { "sliding_vel": 2.5, "admittance_vel": 2.0 },
    { "sliding_vel": 3.0, "admittance_vel": 1.0 },
    { "sliding_vel": 3.0, "admittance_vel": 1.5 },
    { "sliding_vel": 3.0, "admittance_vel": 2.0 },
    { "sliding_vel": 3.5, "admittance_vel": 1.0 },#Fino a qui fatto
    { "sliding_vel": 3.5, "admittance_vel": 1.5 },
    { "sliding_vel": 3.5, "admittance_vel": 2.0 },
    { "sliding_vel": 4.0, "admittance_vel": 1.0 },
    { "sliding_vel": 4.0, "admittance_vel": 1.5 },
    { "sliding_vel": 4.0, "admittance_vel": 2.0 },
    { "sliding_vel": 4.5, "admittance_vel": 1.0 },
    { "sliding_vel": 4.5, "admittance_vel": 1.5 },
    { "sliding_vel": 4.5, "admittance_vel": 2.0 },
    { "sliding_vel": 5.0, "admittance_vel": 1.0 },
    { "sliding_vel": 5.0, "admittance_vel": 1.5 },
    { "sliding_vel": 5.0, "admittance_vel": 2.0 }
]

param_sets_test = [
    {"kp_sliding": 5.0,  "kd_sliding": 5.0 }
]

# === FUNZIONI ===

rosbag_proc = None  # globale per tenere traccia del processo

def start_rosbag(topics, bag_name="sim_data"):
    global rosbag_proc
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    bag_path = os.path.expanduser(f"~/sim_logs/{bag_name}_{timestamp}.bag")
    os.makedirs(os.path.dirname(bag_path), exist_ok=True)

    cmd = ["rosbag", "record", "-O", bag_path] + topics
    print(f"🎥 Avvio rosbag: {bag_path}")
    rosbag_proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)  # dai tempo al processo di inizializzarsi

def stop_rosbag():
    global rosbag_proc
    if rosbag_proc is not None:
        print("🛑 Fermando rosbag...")
        rosbag_proc.send_signal(signal.SIGINT)
        rosbag_proc.wait()
        rosbag_proc = None


def configure_controller(params):
    """Imposta i parametri dinamici usando dynparam su control_manager."""
    import json

    full_param_str = json.dumps(params)
    cmd = [
        "rosrun", "dynamic_reconfigure", "dynparam",
        "set", f"/{UAV_NAME}/control_manager/bump_tolerant_controller", full_param_str
    ]

    try:
        subprocess.check_call(cmd)
        print(f"✅ Parametri aggiornati dinamicamente: {params}")
    except subprocess.CalledProcessError as e:
        print(f"❌ Errore durante la configurazione: {e}")

def launch_simulation():
    global terminal_proc
    print(f"\n🚀 Lanciando simulazione...")
    env = os.environ.copy()
    env["UAV_NAME"] = UAV_NAME
    env["RUN_TYPE"] = "simulation"
    env["UAV_TYPE"] = "naki"

    terminal_proc = subprocess.Popen([
        "gnome-terminal", "--title", f"Sim {UAV_NAME}",
        "--", "bash", "-c",
        "cd ~/mrs_ws/src && ./bump_tolerant_control/tmux/start.sh;"
    ], env=env)

def call_goto():
    time.sleep(20)
    print("📡 Inviando comando goto...")
    goal = "goal: [-30.0, 30.0, 1.35, 0.0]"
    subprocess.call([
        "rosservice", "call", f"/{UAV_NAME}/control_manager/goto", goal
    ])

def call_goto_variable_position(angle):
    time.sleep(40)
    print("📡 Inviando comando goto...")
    goal = f"goal: [-30.0, 30.0, 1.35, {angle}]"
    subprocess.call([
        "rosservice", "call", f"/{UAV_NAME}/control_manager/goto", goal
    ])

def wait_for_controller_to_turn_off(timeout=150):
    topic = f"/{UAV_NAME}/control_manager/bump_tolerant_controller/switch_controller_command"
    print(f"⏳ Aspetto che arrivi 'True' su {topic} (timeout {timeout}s)...")
    start = time.time()
    while time.time() - start < timeout:
        try:
            msg = rospy.wait_for_message(topic, Bool, timeout=1.0)
            if msg.data:
                print("🟢 Cambio controller confermato!")
                return True
        except rospy.ROSException:
            # timeout intermedio, riprova
            pass
    return False

def kill_simulation():
    socket = "mrs"
    session = "simulation"

    # 1) Kill processi nei pane
    try:
        out = subprocess.check_output([
            "tmux", "-L", socket,
            "list-panes", "-s", "-F", "#{pane_pid} #{pane_current_command}"
        ], stderr=subprocess.DEVNULL).decode().splitlines()
    except subprocess.CalledProcessError:
        print("❌ impossibile elencare i pane tmux (socket mrs).")
        return

    for line in out:
        parts = line.split(maxsplit=1)
        if len(parts) != 2:
            continue
        pid_str, cmd = parts
        if "tmux" in cmd:
            continue
        try:
            pid = int(pid_str)
            print(f"🗡  killProcessRecursive {pid}  (cmd: {cmd})")
            subprocess.check_call(["/usr/bin/killProcessRecursive", str(pid)])
            print("   ✔️  ucciso")
        except Exception as e:
            print("   ❌  errore:", e)

    # 2) Forza chiusura della sessione tmux, se ancora esiste
    try:
        sessions = subprocess.check_output(
            ["tmux", "-L", socket, "list-sessions"],
            stderr=subprocess.DEVNULL
        ).decode()
        if session in sessions:
            subprocess.check_call(["tmux", "-L", socket, "kill-session", "-t", session])
            print("🛑 Sessione tmux chiusa completamente.")
        else:
            print("⚠️ Nessuna sessione tmux da chiudere.")
    except subprocess.CalledProcessError:
        print("⚠️ tmux non è in esecuzione o nessuna sessione da chiudere.")


def wait_for_ros_master(timeout=60):
    """Ritorna True appena rostopic list torna 0, o False se timeout."""
    print("⏳ Aspetto che il ROS Master sia up...", end="", flush=True)
    for _ in range(timeout):
        ret = subprocess.call(
            ["rostopic", "list"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        if ret == 0:
            print(" OK")
            return True
        print(".", end="", flush=True)
        time.sleep(1)
    print(" ❌")
    return False

# === MAIN ===

if __name__ == "__main__":
    # cicla sulle configurazioni
    counter = 0

    for i, params in enumerate(param_sets):
        equal_sim = 3
        for j in range (equal_sim):
            print(f"\n========== Simulazione {i * equal_sim + j + 1}/{len(param_sets) * equal_sim} ==========")
            print(f"\n================ Simulazione {i+1}.{j+1}/{len(param_sets)}.{equal_sim} ================")

            start_problem = True

            while start_problem: 
                launch_simulation()
                print(f"Lancio simulazione con parametri: {params}")

                # 1) Aspetta che roscore sia partito nella tmux
                if not wait_for_ros_master(15):
                    print("❌ ROS Master non è partito, abort!")
                    kill_simulation()
                    start_problem = False
                else :
                    start_problem = False
            

            bag_name = f"sim_{i+1}_{j+1}_{params}"
            start_rosbag([
                f"/{UAV_NAME}/estimation_manager/gps_baro/odom",
                f"/{UAV_NAME}/external_wrench_estimator/force_components_filt",
                f"/{UAV_NAME}/control_manager/bump_tolerant_controller/pub_3_normal_velocity_actual",
                f"/{UAV_NAME}/control_manager/bump_tolerant_controller/pub_5_desired_attitude",
                f"/{UAV_NAME}/control_manager/bump_tolerant_controller/pub_6_actual_attitude",
                f"/{UAV_NAME}/control_manager/bump_tolerant_controller/pub_13_sliding_reference",
                f"/{UAV_NAME}/control_manager/bump_tolerant_controller/sliding_phase",
            ], bag_name=bag_name)

            # 2) Ora che il Master è up, inizializza il nodo (solo la PRIMA volta)
            if not rospy.core.is_initialized():  
                rospy.init_node("sim_runner_" + UAV_NAME, anonymous=True)
            
            # 3) Configuro i parametri
            configure_controller(params)

            # 4) Chiamata goto
            call_goto()

            # 5) Aspetto il messaggio di disattivazione
            success = wait_for_controller_to_turn_off()

            if success:
                print("✅ Simulazione completata con successo.")
                counter += 1
                print(f"🏆 Simulazioni completate: {counter}/{i * equal_sim + j + 1}")
            else:
                print("⚠️ Timeout o fallimento nella simulazione.")

            # 6) Chiudo la simulazione
            stop_rosbag()
            time.sleep(2)
            kill_simulation()
            time.sleep(10)

    print("\n🏁 Tutte le simulazioni sono terminate.")