#! /usr/bin/perl


# ***********************************************************************
# *                                                                     *
# * Copyright 2010 Carnegie Mellon University and Intel Corporation *
# * Author: Mike Vande Weghe <vandeweg@cmu.edu>                         *
# *                                                                     *
# ***********************************************************************

use IO::Handle;

$GP[0] = "GNUPLOT0";
$GP[1] = "GNUPLOT1";
open($GP[0] , "| gnuplot -geometry 600x280");
open($GP[1] , "| gnuplot -geometry 600x280");
# set autoflush
$GP[0]->autoflush(1);
$GP[1]->autoflush(1);
# use lines for plots

$setup =<<EOM;
set style data lines
set y2tics
set style line 1 lt 0 lc rgbcolor \"brown\"
set style line 2 lt 0 lc rgbcolor \"red\"
set style line 3 lt 0 lc rgbcolor \"orange\"
set style line 4 lt 0 lc rgbcolor \"yellow\"
set style line 5 lt 0 lc rgbcolor \"green\"
set style line 6 lt 0 lc rgbcolor \"blue\"
set style line 7 lt 0 lc rgbcolor \"violet\"

set style line 8 lt 1 lc rgbcolor \"brown\"
set style line 9 lt 1 lc rgbcolor \"red\"
set style line 10 lt 1 lc rgbcolor \"orange\"
set style line 11 lt 1 lc rgbcolor \"yellow\"
set style line 12 lt 1 lc rgbcolor \"green\"
set style line 13 lt 1 lc rgbcolor \"blue\"
set style line 14 lt 1 lc rgbcolor \"violet\"

set style line 15 lt 1 lc rgbcolor \"black\"
EOM

print {$GP[0]} $setup;
print {$GP[1]} $setup;

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
    $f = sprintf("wamstats32-%04d.csv",$linenum);
#    system("scp wam:/tmp/$f .");
    print "Plotting $f\n";
    
    if (-e $f) {
	$f = "\"$f\"";
	print {$GP[1]} "set title $f\n";
#print {$GP[1]} "replot $f using 3 title \"traj time\"\n"; #traj_time
	print {$GP[1]} "  plot $f using 4  title \"J1 targ\" with lines ls 1\n";
	print {$GP[1]} "replot $f using 5  title \"J1 act\"  with lines ls 8\n";
	print {$GP[1]} "replot $f using 7  title \"J2 targ\" with lines ls 2\n";
	print {$GP[1]} "replot $f using 8  title \"J2 act\"  with lines ls 9\n";

	print {$GP[0]} "set title $f\n";
	print {$GP[0]} "  plot $f using 14           title \"J1 traj\" with lines ls 1\n";
	print {$GP[0]} "replot $f using 6  axes x1y2 title \"J1 PID\" with lines ls 8\n";
	print {$GP[0]} "replot $f using 15           title \"J2 traj\" with lines ls 2\n";
	print {$GP[0]} "replot $f using 9  axes x1y2 title \"J2 PID\" with lines ls 9\n";
	print {$GP[0]} "replot $f using 2 axes x1y2 title \"timescale\" with lines ls 15\n"; # timestep_factor

    } else {
	print "$f not found";
	print {$GP[0]} "clear\n";
	print {$GP[1]} "clear\n";
    }
}


close {$GP[0]};
