## 1.  Environment Configuration (x86)
- Use configured docker
    ```
    xxxx
    ```
    - if can't visualization, using the following command line:
        ```
        sudo docker stop xian_aqc
        sudo docker rm xian_aqc
        xhost +local:root
        xxxx
        ```
- If do not have configured docker, docker preparation as follow
    - Pull nvidia-docker
        ```
        $ sudo docker pull nvidia/cuda:11.4.3-cudnn8-devel-ubuntu20.04
        ```

    - Start docker (docker visualization)
        - xhost +local:root # 出现non-network local connections being added to access control list说明执行成功
        - run docker image
            ```
            sudo docker run --name xian_aqc -itd --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -v /home/ntueee/yangjianbing:/root/code -p 2810:22 -p 2811:2811 -p 2812:2812 -e NVIDIA_DRIVER_CAPABILITIES=compute,utility --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --gpus all --shm-size="12g" --restart=always nvidia/cuda:11.4.3-cudnn8-devel-ubuntu20.04

            # 2810: ssh port
            # 2811: image wireless transmission port
            # 2812: PLC communication port
            ```
        - 注意：
            - 必须在host主机上先运行xhost +local:root
            - 必须在host主机上启动该镜像
            - 必须用sudo docker exec -it xxxx /bin/bash进入镜像

    - Login docker
        ```
        sudo docker exec -it xian_aqc /bin/bash
        ```

    - python3 link  
        ```
        rm -rf /usr/bin/python  
        ln -s /usr/bin/python3 /usr/bin/python
        ```

    - Install dependence
        - apt update && apt install -y libsm6 libxext6
        - apt-get install -y libxrender-dev
        - apt-get install libglib2.0-0
        - pip install -r requirements.txt
    - 测试可视化是否成功  
        ```
        import cv2
        img = cv2.imread("figures/gazebo_show.png")
        cv2.imshow("show", img)
        cv2.waitKey(3000)
        ```

    - Install ssh (Note that enter container first!)
        - `$ apt-get update`
        - `$ apt-get install vim`
        - `$ apt-get install openssh-server`
        - 设置root密码，后续登录会用到: `$ passwd` # password: eee
        - 修改配置文件: `$ vim /etc/ssh/sshd_config`
            ``` 
            #PermitRootLogin prohibit-password
            PermitRootLogin yes
            UsePAM yes 修改为 no
            注释这一行PermitRootLogin prohibit-password
            添加一行PermitRootLogin yes
            UsePAM yes 修改为 no #禁用PAM
            ```
        - 重启ssh服务: `$ service ssh restart`
        - 添加开机启动
            - 新建`power_launch.sh`文件，放到根目录：`/root`下，`power_launch.sh`添加如下内容
                ```
                #!/bin/sh -e
                service ssh start &
                ```
            - 获取读写权限：`chmod 777 /root/power_launch.sh`
            - 编辑`~/.bashrc`: `vim ~/.bashrc`，在下面添加
                ```
                if [ -f /root/power_launch.sh ]; then
                        ./root/power_launch.sh
                fi
                ```

    - [ROS Installation](https://wiki.ros.org/noetic/Installation/Ubuntu)

    - Related Lib
    - rosbridge_suite
        ```
        apt-get update
        apt-get install ros-noetic-rosbridge-suite
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