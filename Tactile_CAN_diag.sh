#! /bin/sh

# Simple script to process a candata0.log file to check for tactile sensor packets

usage()
{
    echo "\nUsage: tactile_can_test.sh file\n" 2>&1
    echo "Dumps info from the OWD CAN log, file, on number of CAN messages for each finger puck and the average time between samples.\n"
    exit 85
}

if [ ! -f "$1" ] ; then
    echo "$1: no such file" 2>&1
    usage
fi

awk \
'BEGIN {time_i = -1; first = 1} \
/TACTILE/ {split($9, gp, "="); \
count[($4 gp[2])]++; \
if (first == 1) \
{first = 0; split($2, time, ":|,"); initial = time[1]*3600000000 + time[2]*60000000 + time[3]*1000000 + time[4]} \
else last_time = $2} \
END {total = 0; \
split($2, time, ":|,"); final = time[1]*3600000000 + time[2]*60000000 + time[3]*1000000 + time[4]; interval = final-initial; \
for (puck = 11; puck <= 14; puck++) \
{print "Puck " puck; \
for (group = 0; group <= 4; group++) \
{print "\t" count[(puck group)]; total = total+count[(puck group)];} \
}
print "Average sample time: " (interval*20/total/1000) "ms"
}' $1 