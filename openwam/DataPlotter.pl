#! /usr/bin/perl

use IO::Handle;

$GP[1] = "GNUPLOT1";
open($GP[1] , "| gnuplot -geometry 600x280");
# set autoflush
$GP[1]->autoflush(1);
# use lines for plots
print {$GP[1]} "set style data lines\n";
print {$GP[1]} "set y2tics\n";

$linenum = -1;
while (<>) {
    if (/b|B|p/) { # back / previous
	$linenum = $linenum-1;
	if ($linenum < 0) {
	    $linenum = 0;
	}
    } elsif (/(\d+)/) {
	$linenum = $1;
    } elsif (/LP/) {
	print {$GP[1]} "set style data linespoints\n";
    } elsif (/P/) {
	print {$GP[1]} "set style data points\n";
    } elsif (/L/) {
	print {$GP[1]} "set style data lines\n";
    } else {
	$linenum = $linenum+1;
    }
    $f = sprintf("wamstats%04d.csv",$linenum);
    print "Plotting $f\n";
    
    if (-e $f) {
	$f = "\"$f\"";
	print {$GP[1]} "set title $f\n";
#print {$GP[1]} "plot $f using 1:2\n"; # timestep_factor
#print {$GP[1]} "replot $f using 1:3\n"; #traj_time
	print {$GP[1]} "plot $f using 1:4 title \"J1 targ\"\n";
	print {$GP[1]} "replot $f using 1:5 title \"J1 act\"\n";
	print {$GP[1]} "replot $f using 1:6 axes x1y2 title \"J1 PID\"\n";
#	print {$GP[1]} "replot $f using 1:7 title \"J2 targ\"\n";
#	print {$GP[1]} "replot $f using 1:8 title \"J2 act\"\n";
#	print {$GP[1]} "replot $f using 1:9 axes x1y2 title \"J2 PID\"\n";
#print {$GP[1]} "replot $f using 1:10\n"; # J3 target
#print {$GP[1]} "replot $f using 1:11\n"; # J3 actual
#print {$GP[1]} "replot $f using 1:12 axes x1y2 title \"J3 PID\"\n";
#print {$GP[1]} "replot $f using 1:13\n"; # J4 target
#print {$GP[1]} "replot $f using 1:14\n"; # J4 actual
#print {$GP[1]} "replot $f using 1:16\n"; # J5 target
#print {$GP[1]} "replot $f using 1:17\n"; # J5 actual
#print {$GP[1]} "replot $f using 1:19\n"; # J6 target
#print {$GP[1]} "replot $f using 1:20\n"; # J6 actual
#print {$GP[1]} "replot $f using 1:22\n"; # J7 target
#print {$GP[1]} "replot $f using 1:23\n"; # J7 actual
	print {$GP[1]} "replot $f using 1:25 axes x1y2 title \"J1 dyn\"\n";
#	print {$GP[1]} "replot $f using 1:26 axes x1y2 title \"J2 dyn\"\n";
#for ($col=3; $col<21; ++$col) {
#  print {$GP[1]} "replot $f using 1:$col\n";
#}


    } else {
	print "$f not found";
	print {$GP[1]} "clear\n";
    }
}


close {$GP[0]};
