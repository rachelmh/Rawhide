#!/bin/bash


gnome-terminal --tab --title="poirot1" --command="bash -c 'cd ~/rawhide/rawhide_ws; source intera_robot1.sh;'"


gnome-terminal --tab --title="captain" --command="bash -c 'cd ~/rawhide/rawhide_ws; source intera_robot2.sh;'"

gnome-terminal --tab --title="tbd" --command="bash -c 'cd ~/rawhide/rawhide_ws; '"
