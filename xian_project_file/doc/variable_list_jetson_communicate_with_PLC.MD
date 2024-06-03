- PLC->ACDS						

    | 序号 | 名字 | 数据类型 | 数据长度 | 地址 | 数据 | 描述 |
    |:--- |:--- |:--- |:--- |:--- |:--- |:---  |
    | 1 | Heart Beat_PLC | WORD | 2 | BYTE0 |  | heart beat value(1 ~ 32767) |
    | 2 | Spreader Size | WORD | 2 | BYTE2 |  | 1-20;2-40,3-45;4-twin20 |
    | 3 | Spreader Lock Signal | WORD | 2 | BYTE4 |  | 0:other;1:lock;2:unlock |
    | 4 | Spreader Land Signal | WORD | 2 | BYTE6 |  | 0:other;1:landing;2:unlanding |
    | 5 | Gantry Position | DINT | 4 | BYTE8 | unit:mm |  |
    | 6 | Trolley Position | DINT | 4 | BYTE12 | unit:mm |  |
    | 7 | Hoist Position | DINT | 4 | BYTE16 | unit:mm |  |
    | 8 | Target Gantry Position | DINT | 4 | BYTE20 | unit:mm |  |
    | 9 | Target Trolley Position | DINT | 4 | BYTE24 | unit:mm |  |
    | 10 | Accs Job ID | DWORD | 4 | BYTE28 |  | use bms job id |
    | 11 | Cmd Index | BYTE | 1 | BYTE32 | 1-255 | 表示当前PLC下发任务的序号，0-默认,1-255递增；发送一次 |
    | 12 | Cmd Type | BYTE | 1 | BYTE33 | 1-255 | 任务类型 堆场（23:抓箱；26:放箱）、集卡（24:抓箱；27:放箱）、AGV(22:抓箱；25:放箱)发送一次 |
    | 13 | Cmd Add Info | BYTE | 1 | BYTE34 | 1-255 | 任务附加信息；备用（地面标记线类型） |
    | 14 | Sea/Land Side single | BYTE | 1 | BYTE35 | 0-集卡侧 1-堆场 | 集卡道为陆侧，堆场为海侧； 实时发送 |
    | 15 | Gantry Speed | INT | 2 | BYTE36 |  | 大车反馈速度 |
    | 16 | Trolley Speed | INT | 2 | BYTE38 |  | 小车反馈速度 |
    | 17 | Hoist Speed | INT | 2 | BYTE40 |  | 起升反馈速度 |
    | 18 | Target Hoist Position | INT | 2 | BYTE42 | unit:mm | 目标起升位置 |
    | 19 | Capture Image Flag | INT | 2 | BYTE44 |  |  |
    | 20 | TDS Left Control_X | INT | 2 | BYTE46 | unit:mm | TDS左侧控制点X坐标 |
    | 21 | TDS Left Control_Y | INT | 2 | BYTE48 | unit:mm | TDS左侧控制点Y坐标 |
    | 22 | TDS Right Control_X | INT | 2 | BYTE50 | unit:mm | TDS右侧控制点X坐标 |
    | 23 | TDS Right Control_Y | INT | 2 | BYTE52 | unit:mm | TDS右侧控制点Y坐标 |
    | 24 | SDS Left Control_X | INT | 2 | BYTE54 | unit:mm | SDS左侧控制点X坐标 |
    | 25 | SDS Left Control_Y | INT | 2 | BYTE56 | unit:mm | SDS左侧控制点Y坐标 |
    | 26 | SDS Right Control_X | INT | 2 | BYTE58 | unit:mm | SDS右侧控制点X坐标 |
    | 27 | SDS Right Control_Y | INT | 2 | BYTE60 | unit:mm | SDS右侧控制点Y坐标 |
    | 28 | AFLS Enable | WORD | 2 | BYTE62 | 0-不启用 1-启用 | 系统使能信号，发送一次 |
    | 29 | State0 | WORD | 2 | BYTE64 | 0-None(运动过程中)；1-收回；2-伸出; 3-故障 | 伸缩盒子0的状态 |
    | 30 | State1 | WORD | 2 | BYTE66 | 0-None(运动过程中)；1-收回；2-伸出; 3-故障 | 伸缩盒子1的状态 |
    | 31 | State2 | WORD | 2 | BYTE68 | 0-None(运动过程中)；1-收回；2-伸出; 3-故障 | 伸缩盒子2的状态 |
    | 32 | State3 | WORD | 2 | BYTE70 | 0-None(运动过程中)；1-收回；2-伸出; 3-故障 | 伸缩盒子3的状态 |
    | 33 | mode0 | WORD | 2 | BYTE72 | 0手动；1自动 | 伸缩盒子0的工作模式 |
    | 34 | mode1 | WORD | 2 | BYTE74 | 0手动；1自动 | 伸缩盒子1的工作模式 |
    | 35 | mode2 | WORD | 2 | BYTE76 | 0手动；1自动 | 伸缩盒子2的工作模式 |
    | 36 | mode3 | WORD | 2 | BYTE78 | 0手动；1自动 | 伸缩盒子3的工作模式 |
    |  |  |  | 80 |  |  |  |

- ACDS->PLC	  

    | 序号 | 名字 | 数据类型 | 数据长度 | 地址 | 数据 | 描述 |
    |:--- |:--- |:--- |:--- |:--- |:--- |:---  |
    | 1 | Cmd Index Fb | BYTE | 1 | BYTE0 | 1-255 | 反馈之前的PLC下发的Index |
    | 2 | Cmd Type  Fb | BYTE | 1 | BYTE1 | 1-255 | 工况代码 |
    | 3 | Heart Beat_AFLS | INT | 2 | BYTE2 | 1-32767 | AFLS返回心跳 |
    | 4 | AFLS Status | INT | 2 | BYTE4 | 1-255 | 工作状态 （0:待机状态，1:相机激活状态,2:做识别,3:任务结束,4:停止任务）    |
    | 5 | Camera Status | INT | 2 | BYTE6 | 0-32767 | 0: 代表正常；1:掉线 |
    | 6 | AFLS Error Code | WORD | 2 | BYTE8 | Error Code | 故障代码，识别故障；（0:错误，1:代表正常，2:图片读取错误，3:角点条件不满足） |
    | 7 | Left Top Diff_X | INT | 2 | BYTE10 | unit:mm | 左上角大车方向像素偏差 |
    | 8 | Left Top Diff_Y | INT | 2 | BYTE12 | unit:mm | 左上角小车方向像素偏差 |
    | 9 | Right Top Diff_X | INT | 2 | BYTE14 | unit:mm | 右上角大车方向像素偏差 |
    | 10 | Right Top Diff_Y | INT | 2 | BYTE16 | unit:mm | 右上角小车方向像素偏差 |
    | 11 | Left Low Diff_X | INT | 2 | BYTE18 | unit:mm | 左下角大车方向像素偏差 |
    | 12 | Left Low Diff_Y | INT | 2 | BYTE20 | unit:mm | 左下角小车方向像素偏差 |
    | 13 | Right Low Diff_X | INT | 2 | BYTE22 | unit:mm | 右下角大车方向像素偏差 |
    | 14 | Right Low Diff_Y | INT | 2 | BYTE24 | unit:mm | 右下角小车方向像素偏差 |
    | 15 | Diff X | INT | 2 | BYTE26 | unit:mm | 目标大车偏差 |
    | 16 | Diff Y | INT | 2 | BYTE28 | unit:mm | 目标小车方向 |
    | 17 | Target Angle  | INT | 2 | BYTE30 | unit:0.001度 | 目标旋转角度 |
    | 18 | mode0 | WORD | 2 | BYTE32 | 0手动；1自动 | 伸缩盒子0的工作模式 |
    | 19 | mode1 | WORD | 2 | BYTE34 | 0手动；1自动 | 伸缩盒子1的工作模式 |
    | 20 | mode2 | WORD | 2 | BYTE36 | 0手动；1自动 | 伸缩盒子2的工作模式 |
    | 21 | mode3 | WORD | 2 | BYTE38 | 0手动；1自动 | 伸缩盒子3的工作模式 |
    | 22 | Spare1 | WORD | 2 | BYTE40 | spare5 |  |
    | 23 | Spare2 | WORD | 2 | BYTE42 | spare6 |  |
    | 24 | Spare3 | WORD | 2 | BYTE44 | spare7 |  |
    | 25 | Spare4 | WORD | 2 | BYTE46 | spare8 |  |
    |  |  |  | 48 |  |  |  |
