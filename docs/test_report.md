# Test Report

> **Historical record below.** It was captured before the release-readiness pass.
> Current validation on 2026-11-16: **45 passed, 5 deselected** for the fast suite and
> **50 passed** for the full suite using `/Users/saiuppalapati/miniforge3/bin/python3`.
> The full suite now includes concrete PyBullet physics and robot-interface tests.

============================= test session starts ==============================
platform darwin -- Python 3.12.12, pytest-9.1.1, pluggy-1.6.0 -- /Users/saiuppalapati/miniforge3/bin/python3
cachedir: .pytest_cache
rootdir: /Users/saiuppalapati/Downloads/Robotics Project
configfile: pyproject.toml
plugins: mock-3.15.1, anyio-4.14.1, asyncio-1.4.0, langsmith-0.11.2
asyncio: mode=Mode.STRICT, debug=False, asyncio_default_fixture_loop_scope=None, asyncio_default_test_loop_scope=function
collecting ... collected 50 items / 3 deselected / 47 selected

tests/test_config.py::test_robot_config_loads PASSED                     [  2%]
tests/test_config.py::test_simulation_config_loads PASSED                [  4%]
tests/test_config.py::test_pid_config_loads PASSED                       [  6%]
tests/test_config.py::test_lqr_config_loads PASSED                       [  8%]
tests/test_dynamics.py::test_model_params_positive PASSED                [ 10%]
tests/test_dynamics.py::test_A_shape PASSED                              [ 12%]
tests/test_dynamics.py::test_A_has_unstable_eigenvalue PASSED            [ 14%]
tests/test_dynamics.py::test_A_has_expected_zero_eigenvalues PASSED      [ 17%]
tests/test_dynamics.py::test_system_controllable PASSED                  [ 19%]
tests/test_dynamics.py::test_B_produces_forward_tilt_response PASSED     [ 21%]
tests/test_dynamics.py::test_A_kinematic_entries PASSED                  [ 23%]
tests/test_dynamics.py::test_gravity_term_positive PASSED                [ 25%]
tests/test_dynamics.py::test_controllability_matrix_rank PASSED          [ 27%]
tests/test_io_and_plots.py::test_save_and_load_run PASSED                [ 29%]
tests/test_io_and_plots.py::test_csv_has_expected_columns PASSED         [ 31%]
tests/test_io_and_plots.py::test_plot_tilt_vs_time PASSED                [ 34%]
tests/test_io_and_plots.py::test_plot_torque_vs_time PASSED              [ 36%]
tests/test_lqr.py::test_lqr_gain_shape PASSED                            [ 38%]
tests/test_lqr.py::test_lqr_closed_loop_stable PASSED                    [ 40%]
tests/test_lqr.py::test_lqr_zero_state_zero_output PASSED                [ 42%]
tests/test_lqr.py::test_lqr_forward_tilt_produces_output PASSED          [ 44%]
tests/test_lqr.py::test_lqr_saturation PASSED                            [ 46%]
tests/test_lqr.py::test_lqr_reset_is_noop PASSED                         [ 48%]
tests/test_lqr.py::test_lqr_k_sign_convention PASSED                     [ 51%]
tests/test_metrics.py::test_metrics_perfect_balance PASSED               [ 53%]
tests/test_metrics.py::test_metrics_failed_run PASSED                    [ 55%]
tests/test_metrics.py::test_metrics_rms_tilt PASSED                      [ 57%]
tests/test_metrics.py::test_metrics_settling_time PASSED                 [ 59%]
tests/test_metrics.py::test_metrics_empty_df PASSED                      [ 61%]
tests/test_physics.py::test_physics_module_importable PASSED             [ 63%]
tests/test_physics.py::test_physics_placeholder PASSED                   [ 65%]
tests/test_pid.py::test_pid_zero_tilt_zero_output PASSED                 [ 68%]
tests/test_pid.py::test_pid_proportional_response PASSED                 [ 70%]
tests/test_pid.py::test_pid_saturation PASSED                            [ 72%]
tests/test_pid.py::test_pid_integral_clamp PASSED                        [ 74%]
tests/test_pid.py::test_pid_reset PASSED                                 [ 76%]
tests/test_pid.py::test_pid_derivative_on_measurement PASSED             [ 78%]
tests/test_pid.py::test_pid_dt_zero_raises PASSED                        [ 80%]
tests/test_robot.py::test_robot_module_importable PASSED                 [ 82%]
tests/test_robot.py::test_robot_placeholder PASSED                       [ 85%]
tests/test_sensors.py::test_tilt_extraction_upright PASSED               [ 87%]
tests/test_sensors.py::test_tilt_extraction_forward_lean PASSED          [ 89%]
tests/test_sensors.py::test_tilt_extraction_backward_lean PASSED         [ 91%]
tests/test_sensors.py::test_tilt_rate_from_angular_velocity PASSED       [ 93%]
tests/test_sensors.py::test_state_to_array PASSED                        [ 95%]
tests/test_sensors.py::test_known_pose_validation PASSED                 [ 97%]
tests/test_sensors.py::test_tilt_extraction_large_angle PASSED           [100%]

=============================== warnings summary ===============================
tests/test_io_and_plots.py::test_save_and_load_run
tests/test_io_and_plots.py::test_csv_has_expected_columns
  /Users/saiuppalapati/Downloads/Robotics Project/src/balancing_robot/analysis/io.py:43: DeprecationWarning: datetime.datetime.utcnow() is deprecated and scheduled for removal in a future version. Use timezone-aware objects to represent datetimes in UTC: datetime.datetime.now(datetime.UTC).
    metadata["saved_at"] = datetime.utcnow().isoformat()

-- Docs: https://docs.pytest.org/en/stable/how-to/capture-warnings.html
================= 47 passed, 3 deselected, 2 warnings in 1.54s =================
