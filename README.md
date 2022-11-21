# MR-msg-transfer
msg transfer for multi-robot
## Requirement
* informer
make sure informer is installed <br />
https://github.com/jiangpin-legend/informer2 <br />

```
git clone git@github.com:jiangpin-legend/informer2.git
cd informer2
pip install -e .
```

## How to use
### Edge 
```
python3 edge.py
```
#### config
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
* recv
  * is_client: False
* send
  * is_client: True
  * target_ip:(ip of target robot,when you need to send message to multi robots,you need to have multi config file)
