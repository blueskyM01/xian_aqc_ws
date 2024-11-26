## 1. Retractable box (192.168.1.11:3810)
- Get PLC heart beat
    - `rosparam get /xian_aqc_dynamic_parameters_server/xian_plc_heart_beat`
- PLC error clear
    ```
    rosparam set /xian_aqc_dynamic_parameters_server/xian_plc_error_clear 0
    rosparam set /xian_aqc_dynamic_parameters_server/xian_plc_error_clear 1
    ```

- Retractable box operational mode
    ```
    manual mode
    rosparam set /xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode0 0
    rosparam set /xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode1 0
    rosparam set /xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode2 0
    rosparam set /xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode3 0

    auto mode
    rosparam set /xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode0 1
    rosparam set /xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode1 1
    rosparam set /xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode2 1
    rosparam set /xian_aqc_dynamic_parameters_server/xian_acds_send_to_retrable_box_mode3 1
    ```

- Get retractable box state
    ```
    rosparam get /xian_aqc_dynamic_parameters_server/xian_retrable_box_state0
    rosparam get /xian_aqc_dynamic_parameters_server/xian_retrable_box_state1
    rosparam get /xian_aqc_dynamic_parameters_server/xian_retrable_box_state2
    rosparam get /xian_aqc_dynamic_parameters_server/xian_retrable_box_state3
    ```

- Get ultrasonic state
    ```
    rosparam get /xian_aqc_dynamic_parameters_server/xian_ultrasonic0_0
    rosparam get /xian_aqc_dynamic_parameters_server/xian_ultrasonic1_0
    rosparam get /xian_aqc_dynamic_parameters_server/xian_ultrasonic2_0
    rosparam get /xian_aqc_dynamic_parameters_server/xian_ultrasonic3_0
    rosparam get /xian_aqc_dynamic_parameters_server/xian_ultrasonic0_1
    rosparam get /xian_aqc_dynamic_parameters_server/xian_ultrasonic1_1
    rosparam get /xian_aqc_dynamic_parameters_server/xian_ultrasonic2_1
    rosparam get /xian_aqc_dynamic_parameters_server/xian_ultrasonic3_1
    ```