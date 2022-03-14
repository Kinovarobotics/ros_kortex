#!/bin/bash   

NUM=$((1 + $RANDOM % 2))
echo $NUM
NUM=2
if (( $NUM == 1));
then
    echo "pepper, tomato" 
    roslaunch kortex_scripts spawn_qr_grasp_block.launch x:=0.36 type:='tomato' object_name:='qr_tomato_block'
    roslaunch kortex_scripts spawn_qr_grasp_block.launch x:=0.26 type:='pepper' object_name:='qr_pepper_block'
else
    echo "tomato, pepper"
    roslaunch kortex_scripts spawn_qr_grasp_block.launch x:=0.36 type:='pepper' object_name:='qr_pepper_block'
    roslaunch kortex_scripts spawn_qr_grasp_block.launch x:=0.26 type:='tomato' object_name:='qr_tomato_block'
fi