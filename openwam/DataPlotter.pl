#! /usr/bin/perl

use IO::Handle;

$GP[1] = "GNUPLOT1";
open($GP[1] , "| gnuplot -geometry 600x280");
# set autoflush
$GP[1]->autoflush(1);
# use lines for plots
print {$GP[1]} "set style data linespoints\n";
print {$GP[1]} "plot \"wamstats.csv\" using 1:2\n"; # timestep_factor
print {$GP[1]} "replot \"wamstats.csv\" using 1:3\n"; #traj_time
print {$GP[1]} "replot \"wamstats.csv\" using 1:4\n"; # J1 target
print {$GP[1]} "replot \"wamstats.csv\" using 1:5\n"; # J1 actual
print {$GP[1]} "replot \"wamstats.csv\" using 1:7\n"; # J2 target
print {$GP[1]} "replot \"wamstats.csv\" using 1:8\n"; # J2 actual
#print {$GP[1]} "replot \"wamstats.csv\" using 1:9\n"; # J2 pid_torq
print {$GP[1]} "replot \"wamstats.csv\" using 1:10\n"; # J3 target
print {$GP[1]} "replot \"wamstats.csv\" using 1:11\n"; # J3 actual
print {$GP[1]} "replot \"wamstats.csv\" using 1:13\n"; # J4 target
print {$GP[1]} "replot \"wamstats.csv\" using 1:14\n"; # J4 actual
print {$GP[1]} "replot \"wamstats.csv\" using 1:16\n"; # J5 target
print {$GP[1]} "replot \"wamstats.csv\" using 1:17\n"; # J5 actual
print {$GP[1]} "replot \"wamstats.csv\" using 1:19\n"; # J6 target
print {$GP[1]} "replot \"wamstats.csv\" using 1:20\n"; # J6 actual
#print {$GP[1]} "replot \"wamstats.csv\" using 1:22\n"; # J7 target
#print {$GP[1]} "replot \"wamstats.csv\" using 1:23\n"; # J7 actual

#for ($col=3; $col<21; ++$col) {
#  print {$GP[1]} "replot \"wamstats.csv\" using 1:$col\n";
#}

while (<>) {
}

close {$GP[0]};
