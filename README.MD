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

## 4. TCP variable list 
- For client: note that **write** first, then **read**.
- For server: note that **read** first, then **write**.
```
struct RetractableBoxToServer
{
    int tcp_retrable_box_heart_beat = 0;    // tcp client heart beat
    int retractable_motion_flag = 0;        // 0: stop 1: in processing
    int retractable_box_status = 0;         // 0: extent, 1: retract
    int error_code = 9000;                  // 9001: approximated switch error
};
```

```
struct ServerToRetractableBox
{
    int tcp_server_heart_beat;       // heart beat
    int auto_manual_switch_flag;     // 0: auto; 1:manual
};
```