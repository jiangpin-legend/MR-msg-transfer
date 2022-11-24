# MR-msg-transfer
多机ros消息转发

## Requirement
### informer
通过下列指令安装informer <br />
https://github.com/jiangpin-legend/informer2 <br />

```
git clone git@github.com:jiangpin-legend/informer2.git
cd informer2
pip install -e .
```
## 支持消息类型
* submap
* tf
* img
* pointcloud

## 机器人编号
默认从1号开始编号，即中心机器人为1号

## How to use
### Edge 
中心端机器人运行edge.py
```
cd scripts
python3 edge.py
```
#### config
中心端机器人需要配置自身的接收的配置文件:config-edge-recv.yaml<br />
以及发送到其他机器人的发送文件:config/config-edge-send-$(robot_id).yaml,需要将各机器人的ip正确设定

* config file need to be set 
  * config/config-edge-recv.yaml
  * config/config-edge-send-1.yaml
  * config/config-edge-send-2.yaml
  * config/config-edge-send-3.yaml
* recv
  * is_client: False(接收tcp-ip请求，为server)
* send
  * is_client: True(发送tcp-ip请求，为client)
  * target_ip:(发送到的机器人的ip)
  
### robot
编队普通机器人运行 robot_$(robot_id).py<br />
以下命令需要在各个机器人内根据机器人id运行不同的文件
```
cd scripts
python3 robot_2.py
python3 robot_3.py
```

#### config
普通机器人需要配置发送和接收两个配置文件，主要需要将 config/config-edge-send-$(robot_id).yaml中的target_ip设置为中心机器人的ip
* config file need to be set 
  * config/config-edge-recv-$(robot_id).yaml
  * config/config-edge-send-$(robot_id).yaml
* recv
  * is_client: False(接收tcp-ip请求，为server)
* send
  * is_client: True(发送tcp-ip请求，为client)
  * target_ip:(发送到的机器人的ip)
