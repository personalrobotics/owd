pr_msgs/Action [] actions  # the actions that this applet is ready 
                           #   to perform NOW

uint8 state                # the applet state (one of the values below)

int last_command_id        # the id number of the last command (good or bad)

string info                # any additional human-readable information

uint8 state_idle=0
uint8 state_busy=1
uint8 state_error=2

