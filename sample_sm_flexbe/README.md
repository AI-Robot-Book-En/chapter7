# sample_sm_flexbe

Sample programs from Section 7.3
This repository provides a two-state state machine implementation using FlexBE.

## Execution

1. Launch the `FlexBE WebUI`.
  ```console
  $ ros2 launch flexbe_webui flexbe_full.launch.py
  ```

> [!NOTE]
> If the `FlexBe WebUI` does not start, some dependent packages may not be installed.
In that case, run `pip3 install -r requires.txt` inside the `flexbe_webui` directory.

2. The `Behavior Dashboard` will be displayed.
![](../docs/sample_sm_flexbe/01_behavior_dashboard.png)

3. Click `Load Behavior` to display the list of available behaviors on the right-hand side.
![](../docs/sample_sm_flexbe/02_load_behavior.png)

4. From the list, select the behavior named `Sample Behavior`.
![](../docs/sample_sm_flexbe/03_loaded_behavior.png)

5. Move to the `Statemachine Editor` and inspect the state machine.
![](../docs/sample_sm_flexbe/04_statemachine_editor.png)

6. Move to `Runtime Control` to execute the state machine.
First, set the value of `max_eat`.

> [!NOTE]
> `max_eat`represents the number of sweets the user can eat.
This value can be freely adjusted.

![](../docs/sample_sm_flexbe/05_runtime_control.png)

7. Next, click `Start Execution` to begin execution.

| Search State | Eat State |
| --- | --- |
| ![](../docs/sample_sm_flexbe/06_search.png) | ![](../docs/sample_sm_flexbe/07_eat.png) |

8. Example output from the execution terminal:
  ```console
  [00:50:45] Onboard engine is ready.
  [00:50:50] --> Mirror - received updated structure with checksum id = 551566305
  [00:50:50] Activate mirror for behavior id = 551566305 ...
  [00:50:50] --> Preparing new behavior...
  [00:50:50] Executing mirror ...
  [00:50:50] Onboard Behavior Engine starting [Sample Behavior : 551566305]
  [00:50:52] Searching for snacks
  [00:50:52] Snacks found!
  [00:50:53] Eating one piece of snack!
  [00:50:53] I have eaten 1 pieces of snack so far!
  [00:50:54] Searching for snacks
  [00:50:54] Snacks found!
  [00:50:55] Eating one piece of snack!
  [00:50:55] I have eaten 2 pieces of snack so far!
  [00:50:56] Searching for snacks
  [00:50:56] Snacks found!
  [00:50:57] Eating one piece of snack!
  [00:50:57] I have eaten 3 pieces of snack so far!
  [00:50:58] Searching for snacks
  [00:50:58] Snacks found!
  [00:50:59] Eating one piece of snack!
  [00:50:59] I have eaten 4 pieces of snack so far!
  [00:51:00] Searching for snacks
  [00:51:00] Snacks found!
  [00:51:01] Eating one piece of snack!
  [00:51:01] I have eaten 5 pieces of snack so far!
  [00:51:02] Searching for snacks
  [00:51:02] I am already full...
  [00:51:02] PreemptableStateMachine 'Sample Behavior' spin() - done with outcome=finished
  [00:51:02] No behavior active.
  [00:51:02] [92m--- Behavior Mirror ready! ---[0m
  ```


## List of States

* [search_state.py](sample_sm_flexbe_states/sample_sm_flexbe_states/search_state.py):
  * Implementation of the state for searching for snacks

* [eat_state.py](sample_sm_flexbe_states/sample_sm_flexbe_states/eat_state.py):
  * Implementation of the state for eating a found snack

* [grasp_state.py](sample_sm_flexbe_states/sample_sm_flexbe_states/grasp_state.py):
  * Implementation of the state for grasping a found snack (advanced version)


## List of Behaviors

* [sample_behavior_sm.py](sample_sm_flexbe_behaviors/sample_sm_flexbe_behaviors/sample_behavior_sm.py):
  * State machine for the Sample task, including food searching and eating states


## Help

## Authors

HAGIWARA Yoshinobu


## History


## License

Copyright (c) 2025, HAGIWARA Yoshinobu, VALENTIN CARDENAS Keith and SIM Jiahao.  
All rights reserved.  
This project is licensed under the Apache License 2.0 license found in the LICENSE file in the root directory of this project.

## References

