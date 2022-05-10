# if ! rosnode list >/dev/null 2>&1; then
#     echo "roscore isn't running"
# elif [ $(rosnode list | grep -v rosout -c) == 0 ]; then
#     echo "Just roscore is running"

# fi

if [ ! $(rosnode list >/dev/null 2>&1) ] || [ $(rosnode list | grep -v rosout -c) == 0 ]; then
    echo "[$waffle_id]: Launching ROS..."
else
    echo "It looks like ROS is already running, do you want to Just roscore."
fi