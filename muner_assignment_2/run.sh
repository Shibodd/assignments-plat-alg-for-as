set -e

ASSIGNMENT_FOLDER=$(dirname $0)

BIN_NAME=ekf_node
BIN=$ASSIGNMENT_FOLDER/devel/lib/ekf/$BIN_NAME
BAG=$ASSIGNMENT_FOLDER/data/2022-07-26-12-45-29.bag

$BIN &
sleep 1
rosbag play $BAG

wait

python3 plotter.py