# MR-msg-transfer
msg transfer for multi-robot

## Requirement
### informer
make sure informer is installed <br />
https://github.com/jiangpin-legend/informer2 <br />

```
git clone git@github.com:jiangpin-legend/informer2.git
cd informer2
pip install -e .
```
## Surport message
* submap
* tf
 
## How to use
### Edge 
```
python3 edge.py
```
#### config
* config file need to be set 
  * config/config-edge-recv.yaml
  * config/config-edge-send-1.yaml
  * config/config-edge-send-2.yaml
  * config/config-edge-send-3.yaml
* recv
  * is_client: False
* send
  * is_client: True
  * target_ip:(ip of target robot,when you need to send message to multi robots,you need to have multi config file)
  
### robot
```
python3 robot_1.py
python3 robot_2.py
```

#### config
* config file need to be set 
  * config/config-edge-recv-$(robot_id).yaml
  * config/config-edge-send-$(robot_id).yaml
* recv
  * is_client: False
* send
  * is_client: True
  * target_ip:(ip of target robot,when you need to send message to multi robots,you need to have multi config file)
