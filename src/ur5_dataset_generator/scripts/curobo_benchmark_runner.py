import yaml
import time
import torch
import csv
import os

from curobo.geom.types import Cuboid, Cylinder, Sphere
from curobo.types.math import Pose
from curobo.types.robot import RobotConfig
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig
from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig
from curobo.geom.sdf.world import WorldConfig

def load_dataset(yaml_path):
    print(f"[*] Ładowanie danych z {yaml_path}...")
    with open(yaml_path, "r") as file:
        return yaml.safe_load(file)

def build_curobo_world(dataset_obstacles):
    print("[*] Budowanie wirtualnego środowiska przeszkód na GPU...")
    obstacles = []
    
    for obs in dataset_obstacles:
        obs_id = obs["id"]
        dims = obs["dimensions"]
        pos = obs["position"]
        pose = Pose(position=torch.tensor(pos, dtype=torch.float32, device="cuda"), 
                    quaternion=torch.tensor([1.0, 0.0, 0.0, 0.0], dtype=torch.float32, device="cuda"))
        
        obs_type = obs.get("type", "BOX")
        
        if obs_type == "BOX":
            obstacles.append(Cuboid(name=obs_id, pose=pose, dims=dims))
        elif obs_type == "CYLINDER":
            obstacles.append(Cylinder(name=obs_id, pose=pose, radius=dims[0], height=dims[1]))
        elif obs_type == "SPHERE":
            obstacles.append(Sphere(name=obs_id, pose=pose, radius=dims[0]))
            
    return WorldConfig(cuboid=obstacles)

def main():
    tensor_args = {"device": torch.device("cuda:0"), "dtype": torch.float32}
    
    # Ścieżka do Twojego pliku YAML z drzewa katalogów (zakładając odpalanie z głównego folderu przestrzeni roboczej)
    yaml_file = "benchmark_queries.yaml" 
    if not os.path.exists(yaml_file):
        print(f"[BŁĄD] Nie znaleziono pliku {yaml_file}. Uruchom skrypt będąc w folderze głównym (custom_benchmark_ws).")
        return

    dataset = load_dataset(yaml_file)
    world_config = build_curobo_world(dataset.get("obstacles", []))
    
    print("[*] Inicjalizacja modelu UR5 na rdzeniach CUDA...")
    robot_config = RobotConfig.from_basic("ur5e") 
    
    motion_gen_config = MotionGenConfig.load_from_robot_config(robot_config, world_config, tensor_args=tensor_args)
    motion_gen = MotionGen(motion_gen_config)
    motion_gen.warmup() 
    
    print("\n================== START BENCHMARKU CUROBO ==================")
    plan_config = MotionGenPlanConfig(max_attempts=10, timeout=10.0)
    
    results = []
    
    if "queries" in dataset:
        for query_item in dataset["queries"]:
            q_name = list(query_item.keys())[0]
            q_data = query_item[q_name]
            
            q_start = torch.tensor(q_data["start"], **tensor_args).unsqueeze(0)
            q_goal = torch.tensor(q_data["goal"], **tensor_args).unsqueeze(0)
            
            print(f"--> Planowanie zapytania: {q_name}...")
            
            t_start = time.time()
            result = motion_gen.plan_single(q_start, q_goal, plan_config)
            t_end = time.time()
            
            process_time = t_end - t_start
            success = result.success.item()
            path_len = result.interpolated_plan.shape[1] if success else 0
            
            print(f"    Sukces: {success} | Czas: {process_time:.4f}s | Długość ścieżki: {path_len}")
            
            results.append({
                "query": q_name,
                "success": success,
                "process_time": process_time,
                "path_length": path_len
            })
            
    print("================== KONIEC BENCHMARKU ==================\n")
    
    # --- ZAPIS DO CSV ---
    csv_filename = "curobo_results.csv"
    with open(csv_filename, mode='w', newline='') as csv_file:
        fieldnames = ['query', 'success', 'process_time', 'path_length']
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        
        writer.writeheader()
        for row in results:
            writer.writerow(row)
            
    print(f"[*] Pomyślnie wyeksportowano wyniki do pliku: {csv_filename}")

if __name__ == "__main__":
    main()