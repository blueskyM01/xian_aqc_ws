#!/bin/bash
while [ 1 ] ; do
sleep 30
    if [ $(ps -ef|grep zpmc_error_restart_log |grep -v grep|wc -l) -eq 0 ] ; then # 将exe_name替换成你想要监测的可执行程序名字
        sleep 1;
        echo "[`date +%F\ %T`] zpmc_error_restart_log is offline, try to restart..." >> ./logs/check_es.log;
        roslaunch /root/code/zpmc_unloading_loading_ws/src/zpmc_utils_pkg/launch/zpmc_unloading_launch.launch &  # 将exe_name替换成你想要监测的可执行程序名字
    else
        rm -rf /root/.ros/*
        echo "[`date +%F\ %T`] rosmaster is online...";
    fi
done
