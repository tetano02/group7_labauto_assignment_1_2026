from __future__ import annotations

import os
import time
from pathlib import Path

import numpy as np
import yaml
from scipy.io import loadmat

from labauto import MuJoCoMechanicalSystem
from labauto import TrapezoidalMotionLaw
from labauto import loadController

# File che simula nuovamente a livello grafico l'esperimento di validazione con segnale chirp,
# per verificare che i risultati siano coerenti con quelli ottenuti in Matlab.
# In particolare è utile per vedere se sbatte contro il finecorsa in alcune situazioni e per settare i parametri limite

# Nome del file da testare nuovamente a livello grafico
TEST_FILE_NAME = "validation_chirp_experiment_joint1_20260401111837.mat"

MODEL_NAME = "gantry_portal_sea_soft"
BASE_DIR = Path(__file__).resolve().parent
MODEL_DIR = BASE_DIR / MODEL_NAME
TESTS_DIR = MODEL_DIR / "tests"


def _load_test_data(file_path: Path) -> dict[str, np.ndarray | int | float]:
    mat = loadmat(file_path, squeeze_me=True)

    time_vector = np.atleast_1d(np.asarray(mat["time"], dtype=float))
    chirp_signal = np.atleast_1d(np.asarray(mat["chirp_signal"], dtype=float))
    joint_position = np.asarray(mat["joint_position"], dtype=float)
    joint_velocity = np.asarray(mat["joint_velocity"], dtype=float)
    joint_torque = np.asarray(mat["joint_torque"], dtype=float)

    return {
        "time": time_vector,
        "chirp_signal": chirp_signal,
        "joint_position": joint_position,
        "joint_velocity": joint_velocity,
        "joint_torque": joint_torque,
        "joint_number": int(np.asarray(mat["joint_number"]).squeeze()),
        "f0": float(np.asarray(mat["f0"]).squeeze()),
        "f1": float(np.asarray(mat["f1"]).squeeze()),
        "A": float(np.asarray(mat["A"]).squeeze()),
    }


def main() -> None:
    os.chdir(BASE_DIR)

    test_file = TESTS_DIR / TEST_FILE_NAME
    if not test_file.exists():
        raise FileNotFoundError(f"Test file not found: {test_file}")

    test_data = _load_test_data(test_file)
    chirp_signal = test_data["chirp_signal"]
    joint_number = test_data["joint_number"]

    with open(MODEL_DIR / "initial_control_config.yaml", "r", encoding="utf-8") as file:
        params_yaml = yaml.safe_load(file)
        controller_params = params_yaml["controller"]
        dynamic_params = np.array(params_yaml["model_parameters"])
        motion_law_params = controller_params["motion_law_parameters"]

    robot = MuJoCoMechanicalSystem(xml_path=str(MODEL_DIR / "model_without_vases.xml"))
    robot.initialize()
    robot.show()
    dof = robot.get_input_number()
    Tc = robot.get_sampling_period()

    if joint_number < 0 or joint_number >= dof:
        raise ValueError(f"Invalid joint_number={joint_number} for dof={dof}")

    decentralized_ctrl = loadController(Tc, controller_params, dynamic_params, MODEL_NAME)
    decentralized_ctrl.initialize()
    decentralized_ctrl.set_umax(robot.get_umax())

    measured_output = robot.read_sensor_value()
    q0 = measured_output[:dof]
    dq0 = measured_output[dof:]
    ddq0 = np.zeros(dof)
    initial_reference = np.concatenate((q0, dq0, ddq0))

    joint_torque = robot.read_actuator_value()
    feedforward_action = np.zeros(dof)
    decentralized_ctrl.starting(initial_reference, measured_output, joint_torque, feedforward_action)

    ml = TrapezoidalMotionLaw(motion_law_params, Tc)
    ml.set_initial_condition(q0)
    ml.add_instructions(["pause: 1", f"move: {[0.0] * dof}", "pause: 5"])

    while ml.depending_instructions():
        loop_t0 = time.perf_counter()
        target_q, target_dq, target_ddq = ml.compute_motion_law()
        reference = np.concatenate((target_q, target_dq, target_ddq))
        measured_output = robot.read_sensor_value()
        joint_torque = decentralized_ctrl.compute_control_action(reference, measured_output, feedforward_action)
        robot.write_actuator_value(joint_torque)
        robot.simulate()

        computation_time = time.perf_counter() - loop_t0
        time.sleep(max(0.0, Tc - computation_time))

    for disturbance in chirp_signal:
        loop_t0 = time.perf_counter()
        target_q, target_dq, target_ddq = ml.compute_motion_law()
        reference = np.concatenate((target_q, target_dq, target_ddq))
        measured_output = robot.read_sensor_value()

        feedforward_action[:] = 0.0
        feedforward_action[joint_number] = disturbance
        joint_torque = decentralized_ctrl.compute_control_action(reference, measured_output, feedforward_action)
        robot.write_actuator_value(joint_torque)

        robot.simulate()
        computation_time = time.perf_counter() - loop_t0
        time.sleep(max(0.0, Tc - computation_time))

    print(f"Replaying file: {test_file.name}")
    print(f"joint_number={joint_number}, f0={test_data['f0']}, f1={test_data['f1']}, A={test_data['A']}")

    robot.close()


if __name__ == "__main__":
    main()
