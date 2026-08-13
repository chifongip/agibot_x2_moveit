from pathlib import Path


PACKAGE = Path(__file__).parents[1]
HEADER = PACKAGE / "include" / "agibot_x2_ros2_control" / "x2_system_hardware.hpp"
SOURCE = PACKAGE / "src" / "x2_system_hardware.cpp"


def test_state_streams_use_independent_latest_sample_callback_groups():
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")

    assert "MultiThreadedExecutor" in header
    assert "std::array<rclcpp::CallbackGroup::SharedPtr, 4>" in header
    assert "std::array<std::mutex, 4> state_mutexes_" in header
    assert "std::mutex data_mutex_" not in header
    assert "rclcpp::SensorDataQoS()" in source
    assert "state_qos.keep_last(1)" in source
    assert "options.callback_group = state_callback_groups_[i]" in source
    assert "state_callback(message, i)" in source
    assert "state_mutexes_[group_index]" in source
    assert "state_group_indices_[index] != group_index" in source
    assert "received on the wrong state topic" in source
    assert "executor_->spin()" in source
    assert "executor_->cancel()" in source
    assert "spin_some()" not in source
