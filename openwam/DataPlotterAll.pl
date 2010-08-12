#! /usr/bin/perl

use IO::Handle;

$GP[0] = "GNUPLOT0";
$GP[1] = "GNUPLOT1";
open($GP[0] , "| gnuplot -geometry 600x280");
open($GP[1] , "| gnuplot -geometry 600x280");
# set autoflush
$GP[0]->autoflush(1);
$GP[1]->autoflush(1);
# use lines for plots
print {$GP[0]} "set style data lines\n";
print {$GP[1]} "set style data lines\n";
print {$GP[0]} "set y2tics\n";
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
#    system("scp wam:/tmp/$f .");
    print "Plotting $f\n";
    
    if (-e $f) {
	$f = "\"$f\"";
	print {$GP[1]} "set title $f\n";
#print {$GP[1]} "replot $f using 3 title \"traj time\"\n"; #traj_time
	print {$GP[1]} "  plot $f using 4  title \"J1 targ\"\n";
	print {$GP[1]} "replot $f using 5  title \"J1 act\"\n";
	print {$GP[1]} "replot $f using 7  title \"J2 targ\"\n";
	print {$GP[1]} "replot $f using 8  title \"J2 act\"\n";
	print {$GP[1]} "replot $f using 10 title \"J3 targ\"\n";
	print {$GP[1]} "replot $f using 11 title \"J3 act\"\n";
	print {$GP[1]} "replot $f using 13 title \"J4 targ\"\n";
	print {$GP[1]} "replot $f using 14 title \"J4 act\"\n";
	print {$GP[1]} "replot $f using 16 title \"J5 targ\"\n";
	print {$GP[1]} "replot $f using 17 title \"J5 act\"\n";
	print {$GP[1]} "replot $f using 19 title \"J6 targ\"\n";
	print {$GP[1]} "replot $f using 20 title \"J6 act\"\n";
	print {$GP[1]} "replot $f using 22 title \"J7 targ\"\n";
	print {$GP[1]} "replot $f using 23 title \"J7 act\"\n";

	print {$GP[0]} "set title $f\n";
	print {$GP[0]} "  plot $f using 6            title \"J1 PID\"\n";
	print {$GP[0]} "replot $f using 32 axes x1y2 title \"J1 dyn\"\n";
#	print {$GP[0]} "replot $f using 9            title \"J2 PID\"\n";
	print {$GP[0]} "replot $f using 33 axes x1y2 title \"J2 dyn\"\n";
#	print {$GP[0]} "replot $f using 12           title \"J3 PID\"\n";
	print {$GP[0]} "replot $f using 34 axes x1y2 title \"J3 dyn\"\n";
#	print {$GP[0]} "replot $f using 15           title \"J4 PID\"\n";
	print {$GP[0]} "replot $f using 35 axes x1y2 title \"J4 dyn\"\n";
#	print {$GP[0]} "replot $f using 18           title \"J5 PID\"\n";
	print {$GP[0]} "replot $f using 36 axes x1y2 title \"J5 dyn\"\n";
#	print {$GP[0]} "replot $f using 21           title \"J6 PID\"\n";
	print {$GP[0]} "replot $f using 37 axes x1y2 title \"J6 dyn\"\n";
#	print {$GP[0]} "replot $f using 24           title \"J7 PID\"\n";
	print {$GP[0]} "replot $f using 38 axes x1y2 title \"J7 dyn\"\n";
	print {$GP[0]} "replot $f using 2 axes x1y2 title \"timescale\"\n"; # timestep_factor

    } else {
	print "$f not found";
	print {$GP[0]} "clear\n";
	print {$GP[1]} "clear\n";
    }
}


close {$GP[0]};
