#!/usr/bin/env bash
# Send a task to an already running snapshot replay stack.
set -eo pipefail

usage() {
  cat <<'EOF'
Usage: send_simulation_task.sh [COMMAND [--object-id tag:N]
                               [--execute | --saved-plan | --plan-id ID]]

Run without arguments for an interactive menu. Use Up/Down arrows and Enter;
no task numbers or tag IDs need to be entered. Press q to quit.
The menu returns after each command completes, including failed commands.

Commands: pick, place, pick-place, carry-a, carry-b, recover-holding
Actions default to plan-only. Add --execute for simulated motion.
Use --saved-plan to preview the complete action, then execute its returned ID
without intermediate planning. Use --plan-id ID to execute an earlier preview
of the same action; target fields are ignored and the ID is single-use.
These three execution options are mutually exclusive.
Pick and PickPlace require --object-id unless using --plan-id
(small carton: tag:0; grey box: tag:180).
Place uses the held object and the configured table placement target.
Carry A/B use the held object and its configured carry targets.
For a loaded Place capture, run recover-holding once after server startup,
then check that the service reports success before sending Place.

Use the same ROS_DOMAIN_ID as the replay launch (ROS default: 0).
Execution requires the replay launch to have allow_execution:=true.

Examples:
  ROS_DOMAIN_ID=114 ./tools/send_simulation_task.sh pick --object-id tag:180
  ROS_DOMAIN_ID=114 ./tools/send_simulation_task.sh pick --object-id tag:180 --execute
  ROS_DOMAIN_ID=114 ./tools/send_simulation_task.sh place --execute
  ROS_DOMAIN_ID=114 ./tools/send_simulation_task.sh pick-place --object-id tag:0 --execute
  ROS_DOMAIN_ID=114 ./tools/send_simulation_task.sh pick-place --object-id tag:0 --saved-plan
  ROS_DOMAIN_ID=114 ./tools/send_simulation_task.sh pick-place --plan-id '<returned ID>'
  ROS_DOMAIN_ID=114 ./tools/send_simulation_task.sh carry-b --saved-plan
  ROS_DOMAIN_ID=114 ./tools/send_simulation_task.sh recover-holding
EOF
}

# Read only a successful action result, never an ID printed in feedback or logs.
saved_result_id() {
  python3 - "$1" <<'PY'
import re
import sys

import yaml

output = open(sys.argv[1], encoding="utf-8").read()
result = output.rsplit("Result:\n", 1)
if len(result) != 2:
    sys.exit("Preview did not return a successful action result; not executing.")
body, separator, status = result[1].rpartition("Goal finished with status:")
if not separator or status.strip() != "SUCCEEDED":
    sys.exit("Preview did not finish with status SUCCEEDED; not executing.")
try:
    # ros2action indents only the first line of message_to_yaml output.
    fields = yaml.safe_load(body.lstrip())
except yaml.YAMLError:
    sys.exit("Preview returned an unreadable action result; not executing.")
if not isinstance(fields, dict) or fields.get("success") is not True:
    sys.exit("Preview did not return a successful action result; not executing.")
plan_id = fields.get("plan_id")
if not isinstance(plan_id, str) or not re.fullmatch(r"[A-Za-z0-9_.:-]{1,160}", plan_id):
    sys.exit("Preview did not return a valid plan_id; rebuild the action interfaces.")
print(plan_id)
PY
}

# Keep prompts on stderr so command output remains readable and redirectable.
choose() {
  local prompt="$1"
  shift
  local choices=("$@")
  local selected=0 key suffix index
  printf '\n%s (Up/Down, Enter; q to quit)\n' "$prompt" >&2
  while true; do
    for index in "${!choices[@]}"; do
      if [[ "$index" -eq "$selected" ]]; then
        printf '\033[2K  > %s\n' "${choices[index]}" >&2
      else
        printf '\033[2K    %s\n' "${choices[index]}" >&2
      fi
    done
    if ! IFS= read -rsn1 key; then exit 0; fi
    case "$key" in
      '') menu_selected="$selected"; return ;;
      q|Q) exit 0 ;;
      $'\033')
        suffix=""
        IFS= read -rsn2 -t 0.2 suffix || true
        case "$suffix" in
          '[A') selected=$(( (selected + ${#choices[@]} - 1) % ${#choices[@]} )) ;;
          '[B') selected=$(( (selected + 1) % ${#choices[@]} )) ;;
          '') exit 0 ;;
        esac
        ;;
    esac
    printf '\033[%sA' "${#choices[@]}" >&2
  done
}

interactive=false
if [[ $# -eq 0 ]]; then
  interactive=true
  if [[ ! -t 0 || ! -t 2 ]]; then
    echo 'Interactive mode requires a terminal. Use --help for command options.' >&2
    exit 2
  fi
elif [[ "$1" == --help || "$1" == -h ]]; then
  usage
  exit 0
fi

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
while [[ ! -f "$workspace_dir/install/setup.bash" ]]; do
  if [[ "$workspace_dir" == / ]]; then
    echo 'Cannot find workspace install/setup.bash; build the workspace first.' >&2
    exit 2
  fi
  workspace_dir="$(dirname -- "$workspace_dir")"
done
source /opt/ros/humble/setup.bash
source "$workspace_dir/install/setup.bash"
preview_output=""
trap 'if [[ -n "$preview_output" ]]; then rm -f -- "$preview_output"; fi' EXIT

while true; do
  object_id=""
  plan_only=true
  execution_mode=""
  plan_id=""
  if [[ "$interactive" == true ]]; then
    choose 'Task' 'Pick' 'Place' 'Pick and Place' 'Carry A' 'Carry B' 'Restore holding for a loaded Place scene' 'Quit'
    case "$menu_selected" in
      0) task=pick ;;
      1) task=place ;;
      2) task=pick-place ;;
      3) task=carry-a ;;
      4) task=carry-b ;;
      5) task=recover-holding ;;
      6) exit 0 ;;
    esac
    if [[ "$task" == pick || "$task" == pick-place ]]; then
      choose 'Object' 'Small carton' 'Grey box'
      case "$menu_selected" in
        0) object_id=tag:0 ;;
        1) object_id=tag:180 ;;
      esac
    fi
    if [[ "$task" != recover-holding ]]; then
      choose 'Mode' 'Plan only' 'Execute in simulation' 'Plan then execute saved plan in simulation'
      case "$menu_selected" in
        1) execution_mode=execute; plan_only=false ;;
        2) execution_mode=saved-plan ;;
      esac
    else
      printf 'After recovery reports success, select Place from the next menu.\n' >&2
    fi
  else
    task="$1"
    shift
  fi
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --object-id)
        if [[ $# -lt 2 || ! "$2" =~ ^tag:[0-9]+$ ]]; then
          echo '--object-id requires tag:N' >&2
          exit 2
        fi
        object_id="$2"
        shift 2
        ;;
      --execute|--saved-plan|--plan-id)
        if [[ -n "$execution_mode" ]]; then
          echo 'Choose only one of --execute, --saved-plan, or --plan-id' >&2
          exit 2
        fi
        execution_mode="${1#--}"
        if [[ "$execution_mode" == plan-id ]]; then
          if [[ $# -lt 2 || ! "$2" =~ ^[A-Za-z0-9_.:-]{1,160}$ ]]; then
            echo '--plan-id requires a valid saved ID (1-160 letters, digits, _, ., :, or -)' >&2
            exit 2
          fi
          plan_id="$2"
          plan_only=false
          shift 2
        else
          if [[ "$execution_mode" == execute ]]; then plan_only=false; fi
          shift
        fi
        ;;
      --help|-h) usage; exit 0 ;;
      *) echo "Unknown option: $1" >&2; exit 2 ;;
    esac
  done

  case "$task" in
    pick|pick-place)
      if [[ -z "$object_id" && -z "$plan_id" ]]; then
        echo 'Pick requires --object-id tag:N' >&2
        exit 2
      fi
      if [[ "$task" == pick ]]; then
        action_name=/pick_box
        action_type=Pick
      else
        action_name=/pick_place
        action_type=PickPlace
      fi
      goal="{instance_id: '$object_id', plan_only: $plan_only}"
      ;;
    place)
      if [[ -n "$object_id" ]]; then
        echo 'Place uses the held object; omit --object-id' >&2
        exit 2
      fi
      action_name=/place_box
      action_type=Place
      goal="{plan_only: $plan_only}"
      ;;
    carry-a|carry-b)
      if [[ -n "$object_id" ]]; then
        echo 'Carry uses the held object; omit --object-id' >&2
        exit 2
      fi
      action_name=/move_carry_pose
      action_type=MoveCarryPose
      if [[ "$task" == carry-a ]]; then carry_target=0; else carry_target=1; fi
      goal="{target_pose: $carry_target, plan_only: $plan_only}"
      ;;
    recover-holding)
      if [[ -n "$object_id" || -n "$execution_mode" ]]; then
        echo 'recover-holding takes no options' >&2
        exit 2
      fi
      ;;
    *) echo "Unknown command: $task" >&2; usage >&2; exit 2 ;;
  esac
  if [[ -n "$plan_id" ]]; then
    goal="{plan_only: false, plan_id: '$plan_id'}"
  fi

  printf 'Sending %s on ROS_DOMAIN_ID=%s\n' "$task" "${ROS_DOMAIN_ID:-0}"
  command_status=0
  if [[ "$task" == recover-holding ]]; then
    ros2 service call /recover_manipulation_state \
      agibot_x2_manipulation_msgs/srv/RecoverManipulationState \
      '{requested_state: 1}' || command_status=$?
  elif [[ "$execution_mode" == saved-plan ]]; then
    preview_output="$(mktemp)"
    ros2 action send_goal "$action_name" \
      "agibot_x2_manipulation_msgs/action/$action_type" "$goal" --feedback \
      | tee "$preview_output" || command_status=$?
    if [[ "$command_status" -eq 0 ]]; then
      plan_id="$(saved_result_id "$preview_output")" || command_status=$?
    fi
    rm -f -- "$preview_output"
    preview_output=""
    if [[ "$command_status" -eq 0 ]]; then
      printf 'Executing saved %s plan %s\n' "$task" "$plan_id"
      ros2 action send_goal "$action_name" \
        "agibot_x2_manipulation_msgs/action/$action_type" \
        "{plan_only: false, plan_id: '$plan_id'}" --feedback \
        || command_status=$?
    fi
  else
    ros2 action send_goal "$action_name" \
      "agibot_x2_manipulation_msgs/action/$action_type" "$goal" --feedback \
      || command_status=$?
  fi
  if [[ "$interactive" != true ]]; then exit "$command_status"; fi
  if [[ "$command_status" -ne 0 ]]; then
    printf 'Command exited with status %s. You can select another task.\n' "$command_status" >&2
  fi
done
