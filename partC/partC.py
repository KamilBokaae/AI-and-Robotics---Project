import numpy as np
import time
import random
import matplotlib.pyplot as plt
from scipy.optimize import linear_sum_assignment

from sim_ur5.mujoco_env.sim_env import SimEnv
from sim_ur5.motion_planning.motion_executor import MotionExecutor
from sim_ur5.mujoco_env.common.ur5e_fk import forward

results = []

def compute_cost_matrix(starts, targets):
    try:
        cost_matrix = np.zeros((len(starts), len(targets)))
        for i, s in enumerate(starts):
            for j, t in enumerate(targets):
                cost_matrix[i, j] = np.linalg.norm(np.array(s[:2]) - np.array(t[:2]))
        return cost_matrix
    except Exception as e:
        print(f"[Error] Failed to compute cost matrix: {e}")
        return np.array([])

def save_run_result(name, assignments, total_cost, exec_time):
    try:
        result = {
            "name": name,
            "assignments": [((round(s[0], 3), round(s[1], 3)), (round(t[0], 3), round(t[1], 3))) for s, t in assignments],
            "total_distance": round(total_cost, 3),
            "execution_time": round(exec_time, 3)
        }
        results.append(result)
    except Exception as e:
        print(f"[Error] Failed to save result for {name}: {e}")

def print_results_summary(results):
    try:
        print("\n--- Heuristic Comparison Summary ---")
        print("{:<12} {:<15} {:<15}".format("Heuristic", "Total Distance", "Execution Time"))
        for r in results:
            print("{:<12} {:<15} {:<15}".format(r["name"], r["total_distance"], r["execution_time"]))
    except Exception as e:
        print(f"[Error] Failed to print summary: {e}")

def plot_execution_times(results):
    try:
        names = [r["name"] for r in results]
        times = [r["execution_time"] for r in results]
        plt.figure(figsize=(8, 5))
        plt.bar(names, times)
        plt.ylabel("Execution Time (s)")
        plt.title("Execution Time per Heuristic")
        plt.grid(axis="y")
        plt.tight_layout()
        plt.show()
    except Exception as e:
        print(f"[Error] Failed to plot execution times: {e}")

def plot_total_distances(results):
    try:
        names = [r["name"] for r in results]
        distances = [r["total_distance"] for r in results]
        plt.figure(figsize=(8, 5))
        plt.bar(names, distances, color='orange')
        plt.ylabel("Total Euclidean Distance")
        plt.title("Total Distance per Heuristic")
        plt.grid(axis="y")
        plt.tight_layout()
        plt.show()
    except Exception as e:
        print(f"[Error] Failed to plot distances: {e}")

def plot_assignment_arrows(assignments, title):
    try:
        plt.figure(figsize=(8, 8))
        for start, target in assignments:
            sx, sy = start[:2]
            tx, ty = target[:2]
            dx, dy = tx - sx, ty - sy
            length = np.linalg.norm([dx, dy])
            offset = 0.0015
            ratio = (length - offset) / length if length > offset else 1
            hx, hy = sx + dx * ratio, sy + dy * ratio

            plt.annotate('', xy=(hx, hy), xytext=(sx, sy),
                         arrowprops=dict(arrowstyle='->,head_width=0.35,head_length=0.7', color='blue', lw=1.6))

            plt.scatter(sx, sy, s=140, color='green', edgecolors='black', marker='o', zorder=3)
            plt.scatter(tx, ty, s=140, color='red', edgecolors='black', marker='o', zorder=3)

        plt.title(title)
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.grid(True)
        plt.axis("equal")
        plt.tight_layout()
        plt.show()
    except Exception as e:
        print(f"[Error] Failed to plot arrows: {e}")

def plot_distance_vs_time(results):
    try:
        plt.figure(figsize=(6, 6))
        for r in results:
            plt.scatter(r["execution_time"], r["total_distance"], label=r["name"], s=100)
            plt.text(r["execution_time"], r["total_distance"] + 0.2, r["name"], ha='center')
        plt.xlabel("Execution Time (s)")
        plt.ylabel("Total Distance")
        plt.title("Heuristic Trade-off: Distance vs Time")
        plt.grid(True)
        plt.tight_layout()
        plt.show()
    except Exception as e:
        print(f"[Error] Failed to plot distance vs time: {e}")

def get_positions():
    start_positions = [
        [-0.6, -0.50, 0.05], [-0.6, -0.60, 0.05], [-0.6, -0.70, 0.05], [-0.6, -0.80, 0.05], [-0.6, -0.90, 0.05],
        [-0.5, -0.50, 0.05], [-0.5, -0.60, 0.05], [-0.5, -0.70, 0.05], [-0.5, -0.80, 0.05], [-0.5, -0.90, 0.05],
        [-0.6, -0.50, 0.025], [-0.6, -0.60, 0.025], [-0.6, -0.70, 0.025], [-0.6, -0.80, 0.025], [-0.6, -0.90, 0.025],
        [-0.5, -0.50, 0.025], [-0.5, -0.60, 0.025], [-0.5, -0.70, 0.025], [-0.5, -0.80, 0.025], [-0.5, -0.90, 0.025]
    ]
    target_positions = [
        [-1.12, -0.60, 0.045], [-1.12, -0.68, 0.045], [-1.12, -0.76, 0.045],
        [-1.04, -0.68, 0.045], [-0.96, -0.68, 0.045], [-0.88, -0.68, 0.045],
        [-0.80, -0.60, 0.045], [-0.80, -0.682, 0.045], [-0.80, -0.764, 0.045],
        [-1.12, -0.88, 0.045], [-0.96, -0.88, 0.045], [-0.88, -0.88, 0.045],
        [-0.80, -0.88, 0.045], [-1.12, -0.96, 0.045], [-1.12, -1.04, 0.045],
        [-1.04, -1.04, 0.045], [-0.96, -1.04, 0.045], [-0.88, -1.04, 0.045],
        [-0.80, -1.04, 0.045], [-0.96, -0.96, 0.045]
    ]
    return start_positions, target_positions

# Matching functions: no changes needed except for fallback on failure
def euclidean_sorted_matching(starts, targets):
    try:
        cost_matrix = compute_cost_matrix(starts, targets)
        if cost_matrix.size == 0:
            return [], 0
        used_targets = set()
        assignments = []
        total_cost = 0
        for i in range(len(starts)):
            best_j = -1
            min_dist = float("inf")
            for j in range(len(targets)):
                if j not in used_targets and cost_matrix[i][j] < min_dist:
                    min_dist = cost_matrix[i][j]
                    best_j = j
            if best_j != -1:
                assignments.append((starts[i], targets[best_j]))
                used_targets.add(best_j)
                total_cost += min_dist
        return assignments, total_cost
    except Exception as e:
        print(f"[Error] Euclidean matching failed: {e}")
        return [], 0

def hungarian_matching(starts, targets):
    try:
        cost_matrix = compute_cost_matrix(starts, targets)
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        assignments = [(starts[i], targets[j]) for i, j in zip(row_ind, col_ind)]
        total_cost = cost_matrix[row_ind, col_ind].sum()
        return assignments, total_cost
    except Exception as e:
        print(f"[Error] Hungarian matching failed: {e}")
        return [], 0

def greedy_matching(starts, targets):
    try:
        used = [False] * len(targets)
        assignments = []
        total_cost = 0
        for s in starts:
            best_idx = -1
            min_dist = float("inf")
            for j, t in enumerate(targets):
                if not used[j]:
                    dist = np.linalg.norm(np.array(s[:2]) - np.array(t[:2]))
                    if dist < min_dist:
                        min_dist = dist
                        best_idx = j
            if best_idx != -1:
                assignments.append((s, targets[best_idx]))
                used[best_idx] = True
                total_cost += min_dist
        return assignments, total_cost
    except Exception as e:
        print(f"[Error] Greedy matching failed: {e}")
        return [], 0

def random_matching(starts, targets):
    try:
        targets_copy = targets.copy()
        random.shuffle(targets_copy)
        assignments = list(zip(starts, targets_copy))
        total_cost = sum(np.linalg.norm(np.array(s[:2]) - np.array(t[:2])) for s, t in assignments)
        return assignments, total_cost
    except Exception as e:
        print(f"[Error] Random matching failed: {e}")
        return [], 0

def run_heuristic(name, match_fn):
    try:
        starts, targets = get_positions()
        env = SimEnv()
        executor = MotionExecutor(env)
        env.reset(randomize=False, block_positions=starts)

        assignments, total_cost = match_fn(starts, targets)
        print(f"Total {name} Distance: {round(total_cost, 3)}")

        start_time = time.time()
        for i, (start, target) in enumerate(assignments):
            pick_z = 0.2 if i < 10 else 0.16
            put_z = target[2] + 0.06
            executor.pick_up("ur5e_2", start[0], start[1], pick_z)
            executor.put_down("ur5e_2", target[0], target[1], put_z)
        executor.wait(4)
        exec_time = time.time() - start_time

        print(f"Execution Time ({name}): {round(exec_time, 2)}s")
        save_run_result(name, assignments, total_cost, exec_time)
        plot_assignment_arrows(assignments, f"{name} Assignment Arrows")

    except Exception as e:
        print(f"[Error] Failed during heuristic '{name}': {e}")

# Run all
run_heuristic("Euclidean", euclidean_sorted_matching)
run_heuristic("Hungarian", hungarian_matching)
run_heuristic("Greedy", greedy_matching)
run_heuristic("Random", random_matching)

print_results_summary(results)
plot_execution_times(results)
plot_total_distances(results)
plot_distance_vs_time(results)
