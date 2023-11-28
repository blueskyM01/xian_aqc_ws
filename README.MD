## 1.  Environment Configuration
### 1.1 Docker Preparation
- Pull nvidia-docker
```
sudo docker nvcr.io/nvidia/l4t-cuda:11.4.19-devel
```

- Start docker
```
sudo docker run --name xian_aqc -itd --runtime nvidia -e DISPLAY=$DISPLAY -v /home/nvidia/yangjianbing:/root/code -p 2810:22 -p 2820:2820 --restart=always nvcr.io/nvidia/l4t-cuda:11.4.19-devel
```

- Login docker
```
sudo docker exec -it xian_aqc /bin/bash
name root password eee
```

- Python3
```
rm -rf /usr/bin/python
ln -s /usr/bin/python3 /usr/bin/python
```

### 1.2 [ROS Installation](https://wiki.ros.org/noetic/Installation/Ubuntu)

### 1.3 Related Lib
- rosbridge_suite
```
apt-get update
apt-get install ros-noetic-rosbridge-suite
pip install numpy==1.17.4
```

- cv_control lib

- 

## 2. Compile
```
cd /xxxx/xian_aqc_ws
catkin_make
```


```
struct RetractableBoxToServer
{
    int tcp_client_heart_beat_tl;    // tcp client heart beat
    int tcp_client_heart_beat_tr;
    int tcp_client_heart_beat_bl;
    int tcp_client_heart_beat_br;
    int retractable_motion_flag_tl;  // 0: stop 1: in processing
    int retractable_motion_flag_tr;
    int retractable_motion_flag_bl;
    int retractable_motion_flag_br;
    int retractable_box_status_tl;   // 0: extent, 1: retract
    int retractable_box_status_tr;
    int retractable_box_status_bl;
    int retractable_box_status_br;
    int error_code_tl;               // 9001: 
    int error_code_tr;
    int error_code_bl;
    int error_code_br;
};
```

```
struct RetractableBoxReadFromServer
{
    int tcp_server_heart_beat;       // heart beat
    int auto_manual_switch_flag;     // 0: auto; 1:manual
    int PLC_retractable_order;       // 0: extent, 1: retract
};
```